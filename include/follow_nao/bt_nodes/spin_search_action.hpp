#ifndef FOLLOW_NAO__BT_NODES__SPIN_SEARCH_ACTION_HPP_
#define FOLLOW_NAO__BT_NODES__SPIN_SEARCH_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace follow_nao
{

class SpinSearchAction : public BT::StatefulActionNode
{
public:
  SpinSearchAction(
    const std::string & action_name,
    const BT::NodeConfig & conf);

  SpinSearchAction() = delete;

  ~SpinSearchAction();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame", "target", "Target TF frame to search for"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame"),
      BT::InputPort<double>("angular_speed", 0.5, "Angular speed for spinning (rad/s)"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel", "Command velocity topic")
    };
  }

private:
  void stopRobot();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  double angular_speed_;
  std::string target_frame_;
  std::string base_frame_;
};

}  // namespace follow_nao

#endif  // FOLLOW_NAO__BT_NODES__SPIN_SEARCH_ACTION_HPP_
