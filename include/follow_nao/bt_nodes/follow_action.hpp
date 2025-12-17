#ifndef FOLLOW_NAO__BT_NODES__FOLLOW_ACTION_HPP_
#define FOLLOW_NAO__BT_NODES__FOLLOW_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nao_lola_sensor_msgs/msg/sonar.hpp"
#include "nao_lola_sensor_msgs/msg/touch.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace follow_nao
{

class FollowAction : public BT::StatefulActionNode
{
public:
  FollowAction(
    const std::string & action_name,
    const BT::NodeConfig & conf);

  FollowAction() = delete;

  ~FollowAction();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame", "target", "Target TF frame to follow"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame"),
      BT::InputPort<double>("min_distance", 1.0, "Desired distance to target (m)"),
      BT::InputPort<double>("avoidance_distance", 0.5, "Minimum distance to obstacles (m)"),
      BT::InputPort<double>("max_linear_speed", 0.5, "Maximum linear speed (m/s)"),
      BT::InputPort<double>("max_angular_speed", 1.0, "Maximum angular speed (rad/s)"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel", "Command velocity topic"),
      BT::InputPort<std::string>("sonar_topic", "/sensors/sonar", "Sonar topic"),
      BT::InputPort<std::string>("touch_topic", "/sensors/touch", "Touch sensor topic")
    };
  }

private:
  void stopRobot();
  void sonarCallback(const nao_lola_sensor_msgs::msg::Sonar::SharedPtr msg);
  void touchCallback(const nao_lola_sensor_msgs::msg::Touch::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Sonar>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Touch>::SharedPtr touch_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  double min_distance_;
  double avoidance_distance_;
  double max_linear_speed_;
  double max_angular_speed_;
  std::string target_frame_;
  std::string base_frame_;
  
  // Obstacle avoidance state
  double vel_rot_avoidance_;
  bool stop_requested_;
};

}  // namespace follow_nao

#endif  // FOLLOW_NAO__BT_NODES__FOLLOW_ACTION_HPP_
