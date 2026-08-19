"""核心 PD 控制器(纯逻辑,无 ROS 依赖)。

参考 robot_utils/liquid_pouring/controllers.py 的
MultiModalPDController.calculate,只保留核心 PD 逻辑:
    error       = target - current
    derivative  = error - prev_error        (每控制周期误差变化量)
    output      = kp*error + kd*derivative,clip 到 joint_range
近目标 kd 缩小 trick 做成可选开关,默认关。
"""


class PDController:
    """电子秤重量 PD 倾倒控制。"""

    def __init__(self, kp, kd, joint_range, near_target_kd_enable=False,
                 near_target_kd_threshold=10.0, near_target_kd=0.025):
        self.kp = kp
        self.kd = kd
        self.joint_range = joint_range
        self.near_target_kd_enable = near_target_kd_enable
        self.near_target_kd_threshold = near_target_kd_threshold
        self.near_target_kd = near_target_kd
        self.prev_error = 0.0

    def calculate(self, target_weight, current_weight):
        """返回 (output, error)。"""
        error = target_weight - current_weight
        derivative = error - self.prev_error
        kd_eff = self.kd
        if self.near_target_kd_enable and abs(error) <= self.near_target_kd_threshold:
            kd_eff = self.near_target_kd
        output = self.kp * error + kd_eff * derivative
        output = max(self.joint_range[0], min(self.joint_range[1], output))
        self.prev_error = error
        return output, error

    def is_reached(self, error, tolerance):
        """误差是否在允许范围内(达标)。"""
        return abs(error) <= tolerance
