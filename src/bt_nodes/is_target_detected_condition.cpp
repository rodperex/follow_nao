#include "follow_nao/bt_nodes/is_target_detected_condition.hpp"
#include "tf2/exceptions.h"

namespace follow_nao
{

IsTargetDetectedCondition::IsTargetDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = rclcpp::Node::make_shared("is_target_detected_condition");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

IsTargetDetectedCondition::~IsTargetDetectedCondition()
{
}

BT::NodeStatus IsTargetDetectedCondition::tick()
{
  std::string target_frame;
  std::string base_frame;
  double timeout;

  if (!getInput("target_frame", target_frame)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [target_frame]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("base_frame", base_frame)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [base_frame]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("timeout", timeout)) {
    timeout = 0.5;
  }

  try {
    // Try to lookup the transform from base to target
    auto transform = tf_buffer_->lookupTransform(
      base_frame,
      target_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(timeout));

    RCLCPP_DEBUG(node_->get_logger(), "Target detected at (%f, %f, %f)",
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);

    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "Target not detected: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace follow_nao
