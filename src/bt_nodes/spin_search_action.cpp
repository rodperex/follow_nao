#include "follow_nao/bt_nodes/spin_search_action.hpp"
#include "tf2/exceptions.h"

namespace follow_nao
{

SpinSearchAction::SpinSearchAction(
  const std::string & action_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(action_name, conf)
{
  node_ = rclcpp::Node::make_shared("spin_search_action");
  
  std::string cmd_vel_topic;
  if (!getInput("cmd_vel_topic", cmd_vel_topic)) {
    cmd_vel_topic = "/cmd_vel";
  }
  
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10);
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

SpinSearchAction::~SpinSearchAction()
{
  stopRobot();
}

BT::NodeStatus SpinSearchAction::onStart()
{
  if (!getInput("angular_speed", angular_speed_)) {
    angular_speed_ = 0.5;
  }
  
  if (!getInput("target_frame", target_frame_)) {
    target_frame_ = "target";
  }
  
  if (!getInput("base_frame", base_frame_)) {
    base_frame_ = "base_link";
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "Starting search: Spinning at %.2f rad/s looking for '%s'",
    angular_speed_, target_frame_.c_str());
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinSearchAction::onRunning()
{
  // Check if target is detected
  try {
    auto transform = tf_buffer_->lookupTransform(
      base_frame_,
      target_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
    
    RCLCPP_INFO(node_->get_logger(), "Target found! Stopping search.");
    stopRobot();
    return BT::NodeStatus::SUCCESS;
    
  } catch (const tf2::TransformException & ex) {
    // Target not found, keep spinning
    RCLCPP_DEBUG(node_->get_logger(), "Still searching for target...");
    
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = angular_speed_;
    
    cmd_vel_pub_->publish(twist_msg);
    
    return BT::NodeStatus::RUNNING;
  }
}

void SpinSearchAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Search halted");
  stopRobot();
}

void SpinSearchAction::stopRobot()
{
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
  
  cmd_vel_pub_->publish(twist_msg);
}

}  // namespace follow_nao
