"""Joint mapping: Alicia-D → UR10e affine transform with safety clamping.

Pure logic — no rclpy imports.
Per joint (positional: alicia_joint_order[i] maps to ur_joint_order[i]):
    ur_cmd[i] = slave_home[i] + sign[i] * scale[i] * (master_q[i] - master_home[i])

Vendored from ur_teleop/ur_teleop/joint_mapper.py@6f202d2 — kept local so this
package never imports ur_teleop (requirement). Keep in sync with the original.
"""


class JointMapper:
    """Per-joint sign/scale/offset mapping with clamping to safety limits."""

    def __init__(self, mapping_config: dict, master_home: list[float], slave_home: list[float]):
        self._alicia_order = list(mapping_config["alicia_joint_order"])
        self._ur_order = list(mapping_config["ur_joint_order"])
        self._signs = [float(s) for s in mapping_config["sign"]]
        self._scales = [float(s) for s in mapping_config["scale"]]
        safety = mapping_config.get("safety", {})
        self._limits = dict(safety.get("limits", {}))
        self._margin = float(safety.get("clamp_margin_rad", 0.1))

        if len(self._alicia_order) != 6 or len(self._ur_order) != 6:
            raise ValueError("alicia_joint_order and ur_joint_order must each have 6 joints")
        if len(self._signs) != 6 or len(self._scales) != 6:
            raise ValueError("sign and scale must each have 6 values")
        if len(master_home) != 6 or len(slave_home) != 6:
            raise ValueError("master_home and slave_home must have 6 values")

        self._master_home = [float(v) for v in master_home]
        self._slave_home = [float(v) for v in slave_home]

        for i, ur_joint in enumerate(self._ur_order):
            if ur_joint not in self._limits:
                raise ValueError(f"UR joint '{ur_joint}' missing from safety.limits")

    @property
    def ur_joint_order(self) -> list[str]:
        return list(self._ur_order)

    @property
    def alicia_joint_order(self) -> list[str]:
        return list(self._alicia_order)

    @property
    def num_joints(self) -> int:
        return len(self._ur_order)

    def master_to_slave(self, master_q: list[float]) -> list[float]:
        """Alicia positions (rad, alicia_joint_order) → clamped UR commands (rad, ur_joint_order)."""
        if len(master_q) != len(self._alicia_order):
            raise ValueError(f"Expected {len(self._alicia_order)} master joints, got {len(master_q)}")
        ur_cmd = []
        for i in range(self.num_joints):
            raw = (
                self._slave_home[i]
                + self._signs[i] * self._scales[i] * (master_q[i] - self._master_home[i])
            )
            ur_cmd.append(self._clamp(self._ur_order[i], raw))
        return ur_cmd

    def _clamp(self, joint_name: str, value: float) -> float:
        limits = self._limits.get(joint_name)
        if limits is None or len(limits) < 2:
            return value
        lo = float(limits[0]) + self._margin
        hi = float(limits[1]) - self._margin
        return max(lo, min(hi, value))

    def get_master_home(self) -> list[float]:
        return list(self._master_home)

    def get_slave_home(self) -> list[float]:
        return list(self._slave_home)
