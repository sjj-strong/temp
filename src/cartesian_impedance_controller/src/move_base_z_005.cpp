#include <memory>
#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
class BaseZ005Command final : public rclcpp::Node
{
public:
  BaseZ005Command()
  : Node("move_base_z_005")
  {
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/cartesian_impedance_controller/target_pose", rclcpp::SystemDefaultsQoS());
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/tcp_pose_broadcaster/pose", rclcpp::SystemDefaultsQoS(),
        std::bind(&BaseZ005Command::pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "等待当前 TCP 位姿；将只发布 base 坐标系下 Z 轴 +0.05 m 的单次目标");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    if (sent_) {
      return;
    }
    if (pose->header.frame_id != "base") {
      RCLCPP_ERROR(get_logger(), "拒绝发布：当前 TCP 位姿坐标系为 '%s'，预期为 'base'", pose->header.frame_id.c_str());
      rclcpp::shutdown();
      return;
    }

    geometry_msgs::msg::PoseStamped target = *pose;
    target.header.stamp = now();
    target.pose.position.z += 0.05;
    publisher_->publish(target);
    sent_ = true;
    RCLCPP_INFO(get_logger(), "已发布固定目标：base 坐标系 Z 轴 +0.05 m");
    rclcpp::shutdown();
  }

  bool sent_{ false };
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseZ005Command>());
  rclcpp::shutdown();
  return 0;
}
