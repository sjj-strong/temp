"""读取 UR 当前关节位置并更新两个源码配置，不发送运动指令。"""
import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import yaml

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
CONFIG_DIR = Path('/ros2_ws/src/ur_teleop/config')


def ordered_positions(names, positions):
    """根据名称排序，过滤夹爪等额外关节，拒绝不完整或非有限值。"""
    if len(names) != len(positions) or len(names) != len(set(names)):
        raise ValueError('关节名称和位置长度不一致，或存在重复关节')
    mapping = dict(zip(names, positions))
    result = [float(mapping[name]) for name in JOINT_NAMES]
    if not all(math.isfinite(value) for value in result):
        raise ValueError('关节位置包含 NaN 或无穷值')
    return result


def mapping_value(node, key):
    if not isinstance(node, yaml.MappingNode):
        raise ValueError(f'{key} 的父节点必须是 YAML 映射')
    matches = [value for name, value in node.value if name.value == key]
    if len(matches) != 1:
        raise ValueError(f'配置必须包含唯一的 {key} 字段')
    return matches[0]


def replace_slave(text, positions):
    """利用 YAML 源码位置局部替换，保留其余字段、格式和注释。"""
    if len(positions) != 6 or not all(math.isfinite(value) for value in positions):
        raise ValueError('必须提供六个有限关节角，单位 rad')
    node = yaml.compose(text)
    slave = mapping_value(mapping_value(node, 'home'), 'slave')
    if not isinstance(slave, yaml.SequenceNode) or len(slave.value) != 6:
        raise ValueError('home.slave 必须是六个关节角组成的序列')
    if slave.flow_style is not True:
        raise ValueError('home.slave 必须使用当前配置的单行数组格式')
    result = text[:slave.start_mark.index] + json.dumps(positions) + text[slave.end_mark.index:]
    # 再次验证替换后的配置，确保字段含义正确。
    if yaml.safe_load(result)['home']['slave'] != positions:
        raise ValueError('替换后的 home.slave 验证失败')
    return result


def write_configs(paths, positions):
    """先验证两个配置，再备份并原子替换；失败时恢复已经写入的文件。"""
    originals = {path: path.read_text(encoding='utf-8') for path in paths}
    updates = {path: replace_slave(text, positions) for path, text in originals.items()}
    pending, backups, replaced = {}, {}, []
    stamp = str(time.time_ns())
    try:
        for path, text in updates.items():
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=path.parent, delete=False) as stream:
                pending[path] = Path(stream.name)
                stream.write(text)
            os.chmod(pending[path], path.stat().st_mode & 0o777)
        # 避免覆盖取样期间用户对配置的编辑。
        for path, text in originals.items():
            if path.read_text(encoding='utf-8') != text:
                raise RuntimeError(f'配置已被其他进程修改：{path}')
        for path, text in originals.items():
            backup = path.with_name(path.name + '.' + stamp + '.bak')
            backup.write_text(text, encoding='utf-8')
            backups[path] = backup
        for path in paths:
            os.replace(pending[path], path)
            replaced.append(path)
    except Exception:
        for path in replaced:
            os.replace(backups[path], path)
        raise
    finally:
        for temporary in pending.values():
            temporary.unlink(missing_ok=True)
    return backups


def driver_command(robot_ip, ur_type):
    """仅启动状态读取所需的 UR 驱动，保持运动控制器未激活。"""
    return ['ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py',
            f'robot_ip:={robot_ip}', f'ur_type:={ur_type}',
            'use_mock_hardware:=false', 'activate_joint_controller:=false',
            'headless_mode:=false', 'launch_rviz:=false', 'launch_dashboard_client:=false']


def stop_driver(process):
    if process is None:
        return
    # 此进程组由脚本自行创建，不停止其他已有的 ROS 进程。
    for sig, timeout in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def capture(args):
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    rclpy.init(args=[])
    node = rclpy.create_node('capture_slave_home')
    process = None
    try:
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=2):
            if args.no_start:
                raise RuntimeError('未发现 /controller_manager；请先启动 UR 控制器')
            print(f'启动 UR 状态读取：{args.robot_ip} / {args.ur_type}', flush=True)
            process = subprocess.Popen(driver_command(args.robot_ip, args.ur_type), start_new_session=True)
        else:
            print('复用现有 /controller_manager，不切换其控制器', flush=True)
        deadline = time.monotonic() + args.timeout
        # 等状态广播器处于 active 后再订阅，避免使用启动前的样本。
        future = None
        ready = False
        while rclpy.ok() and time.monotonic() < deadline:
            if process and process.poll() is not None:
                raise RuntimeError(f'UR 驱动提前退出，退出码 {process.returncode}')
            if future is None and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
            rclpy.spin_once(node, timeout_sec=0.1)
            if future is not None and future.done():
                response = future.result()
                if any(c.name == 'joint_state_broadcaster' and c.state == 'active'
                       for c in response.controller):
                    ready = True
                    break
                future = None
        if not ready:
            raise TimeoutError('等待 joint_state_broadcaster 激活超时，未修改配置')
        sample = []

        def receive(message):
            try:
                values = ordered_positions(message.name, message.position)
            except (KeyError, ValueError, TypeError):
                return
            sample.append(values)

        subscription = node.create_subscription(JointState, args.topic, receive, qos_profile_sensor_data)
        while rclpy.ok() and not sample and time.monotonic() < deadline:
            if process and process.poll() is not None:
                raise RuntimeError('UR 驱动提前退出，未修改配置')
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(subscription)
        if not sample:
            raise TimeoutError(f'未收到 {args.topic} 中的完整六关节位置，未修改配置')
        return sample[0]
    finally:
        stop_driver(process)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='启动或复用 UR 控制器，读取当前关节角并更新 home.slave')
    parser.add_argument('--xbot-config', type=Path, default=CONFIG_DIR / 'xbot_teleop.yaml')
    parser.add_argument('--alicia-config', type=Path, default=CONFIG_DIR / 'alicia_teleop.yaml')
    parser.add_argument('--robot-ip', help='默认读取两个配置的 cell.robot_ip')
    parser.add_argument('--ur-type', help='默认读取两个配置的 cell.ur_type')
    parser.add_argument('--topic', default='/joint_states')
    parser.add_argument('--timeout', type=float, default=60, help='等待状态的总超时，单位秒')
    parser.add_argument('--no-start', action='store_true', help='只复用已有控制器，不启动新驱动')
    args = parser.parse_args()
    try:
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError('timeout 必须为有限正数')
        paths = [args.xbot_config.resolve(), args.alicia_config.resolve()]
        if len(set(paths)) != 2:
            raise ValueError('必须指定两个不同的配置文件')
        configs = [yaml.safe_load(path.read_text(encoding='utf-8')) for path in paths]
        for key in ('robot_ip', 'ur_type'):
            if getattr(args, key) is None:
                values = [config['cell'][key] for config in configs]
                if values[0] != values[1]:
                    raise ValueError(f'两个配置的 cell.{key} 不一致，请显式指定 --{key.replace("_", "-")}')
                setattr(args, key, str(values[0]))
        # 在启动前验证目标文件结构，错误配置不会触发硬件连接。
        for path in paths:
            replace_slave(path.read_text(encoding='utf-8'), [0.0] * 6)
        positions = capture(args)
        print('当前关节角（rad）：')
        for name, position in zip(JOINT_NAMES, positions):
            print(f'  {name}: {position:.15g}')
        backups = write_configs(paths, positions)
        for path, backup in backups.items():
            print(f'已更新 {path} 的 home.slave；备份：{backup}')
    except (Exception, KeyboardInterrupt) as exc:
        parser.exit(1, f'读取或更新失败：{exc}\n')


if __name__ == '__main__':
    main()
