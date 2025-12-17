#ifndef FOLLOW_NAO__BT_NODES__IS_TARGET_DETECTED_CONDITION_HPP_
#define FOLLOW_NAO__BT_NODES__IS_TARGET_DETECTED_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace follow_nao
{

class IsTargetDetectedCondition : public BT::ConditionNode
{
public:
  IsTargetDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfig & conf);

  IsTargetDetectedCondition() = delete;

  ~IsTargetDetectedCondition();

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame", "target", "Target TF frame to check"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame"),
      BT::InputPort<double>("timeout", 0.5, "Time to wait for detection (seconds)")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace follow_nao

#endif  // FOLLOW_NAO__BT_NODES__IS_TARGET_DETECTED_CONDITION_HPP_
