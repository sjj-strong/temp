import pytest

from pour.pd_controller import PDController


def test_proportional_term():
    pd = PDController(kp=0.01, kd=0.0, joint_range=(-2, 2))
    out, err = pd.calculate(target_weight=100.0, current_weight=90.0)
    assert err == pytest.approx(10.0)
    assert out == pytest.approx(0.1)  # kp*error


def test_derivative_term():
    pd = PDController(kp=0.0, kd=0.5, joint_range=(-5, 5))  # 宽 range 避免限幅干扰
    pd.calculate(target_weight=100.0, current_weight=90.0)  # error=10, prev=0
    out, err = pd.calculate(target_weight=100.0, current_weight=95.0)  # error=5
    assert err == pytest.approx(5.0)
    assert out == pytest.approx(-2.5)  # kd*(5-10), 误差减小 → 负导数 → 减速


def test_clip_to_joint_range():
    pd = PDController(kp=1.0, kd=0.0, joint_range=(-0.5, 0.5))
    out, _ = pd.calculate(target_weight=100.0, current_weight=0.0)
    assert out == 0.5


def test_near_target_kd_disabled_by_default():
    pd = PDController(kp=0.0, kd=0.1, joint_range=(-2, 2))
    pd.calculate(target_weight=100.0, current_weight=95.0)   # error=5, prev=0
    out, _ = pd.calculate(target_weight=100.0, current_weight=95.5)  # error=4.5
    # 误差减小 → 导数 -0.5 → 输出 kd*(-0.5);默认不触发近目标 kd,仍用 0.1
    assert out == pytest.approx(0.1 * (-0.5))


def test_near_target_kd_enabled():
    pd = PDController(kp=0.0, kd=0.1, joint_range=(-2, 2),
                      near_target_kd_enable=True, near_target_kd_threshold=10.0,
                      near_target_kd=0.025)
    pd.calculate(target_weight=100.0, current_weight=95.0)
    out, _ = pd.calculate(target_weight=100.0, current_weight=95.5)
    assert out == pytest.approx(0.025 * (-0.5))  # kd 换成 near_target_kd


def test_is_reached():
    pd = PDController(kp=0.0, kd=0.0, joint_range=(-2, 2))
    assert pd.is_reached(error=0.5, tolerance=1.0)
    assert not pd.is_reached(error=1.5, tolerance=1.0)
