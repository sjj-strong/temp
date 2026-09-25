#!/usr/bin/python3
"""用终端按键发布笛卡尔阻抗控制器的目标位姿。"""

import argparse
import copy
import math
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.utilities import remove_ros_args


TARGET_TOPIC = "/cartesian_impedance_controller/target_pose"
POSE_TOPIC = "/cartesian_impedance_controller/current_pose"
KEYS = {
    "w": ("x", 1), "s": ("x", -1),
    "d": ("y", 1), "a": ("y", -1),
    "r": ("z", 1), "f": ("z", -1),
}


def valid_pose(message, frame):
    position = message.pose.position
    orientation = message.pose.orientation
    values = (
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w,
    )
    quaternion_norm = sum(value * value for value in values[3:])
    return message.header.frame_id == frame and all(math.isfinite(value) for value in values) \
        and quaternion_norm > 1e-16


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=float, default=0.005, help="每次按键的平移量，单位 m，默认 0.005")
    parser.add_argument("--max-lead", type=float, default=0.03, help="目标与当前 TCP 的最大距离，单位 m，默认 0.03")
    parser.add_argument("--frame", default="base", help="当前 TCP 与目标的坐标系，默认 base")
    parser.add_argument("--pose-topic", default=POSE_TOPIC, help="当前 TCP 位姿话题")
    parser.add_argument("--target-topic", default=TARGET_TOPIC, help="控制器目标位姿话题")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if not math.isfinite(args.step) or args.step <= 0 or args.step > 0.01:
        parser.error("--step 必须在 (0, 0.01] m 范围内")
    if not math.isfinite(args.max_lead) or args.max_lead <= 0 or args.max_lead > 0.05:
        parser.error("--max-lead 必须在 (0, 0.05] m 范围内")
    if not sys.stdin.isatty():
        parser.error("需要在交互式终端运行")

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("keyboard_target_pose")
    publisher = node.create_publisher(PoseStamped, args.target_topic, 10)
    latest_pose = None
    latest_time = 0.0
    target = None

    def on_pose(message):
        nonlocal latest_pose, latest_time
        if valid_pose(message, args.frame):
            latest_pose = message
            latest_time = time.monotonic()

    subscription = node.create_subscription(PoseStamped, args.pose_topic, on_pose, 10)
    _ = subscription
    old_terminal = termios.tcgetattr(sys.stdin.fileno())
    print("按键：W/S = X±，D/A = Y±，R/F = Z±；空格 = 保持当前位姿；Q = 退出")
    print(f"每步 {args.step:.3f} m；目标相对当前 TCP 最大偏移 {args.max_lead:.3f} m")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            key = sys.stdin.read(1).lower()
            if key == "q" or key == "\x03":
                break
            if key not in KEYS and key != " ":
                continue
            if latest_pose is None or time.monotonic() - latest_time > 1.0:
                print("\r未收到新鲜且坐标系正确的 TCP 位姿，拒绝发布。")
                continue
            if publisher.get_subscription_count() == 0:
                print("\r控制器目标话题没有订阅者，拒绝发布。")
                continue

            if key == " " or target is None:
                target = copy.deepcopy(latest_pose)
            if key in KEYS:
                axis, direction = KEYS[key]
                setattr(target.pose.position, axis,
                        getattr(target.pose.position, axis) + direction * args.step)

            delta = [
                getattr(target.pose.position, axis) - getattr(latest_pose.pose.position, axis)
                for axis in "xyz"
            ]
            if math.sqrt(sum(value * value for value in delta)) > args.max_lead:
                print("\r目标距离当前 TCP 超过上限，拒绝本次按键。")
                if key in KEYS:
                    setattr(target.pose.position, axis,
                            getattr(target.pose.position, axis) - direction * args.step)
                continue

            target.header.frame_id = args.frame
            target.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(target)
            position = target.pose.position
            print(f"\r目标 [{position.x:.3f}, {position.y:.3f}, {position.z:.3f}] m")
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_terminal)
        print("已退出；控制器仍会保持最后收到的目标位姿。")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
