#include "follow_nao/bt_nodes/follow_action.hpp"
#include "tf2/exceptions.h"
#include <cmath>

namespace follow_nao
{

FollowAction::FollowAction(
  const std::string & action_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(action_name, conf),
  vel_rot_avoidance_(0.0),
  stop_requested_(false)
{
  node_ = rclcpp::Node::make_shared("follow_action");
  
  std::string cmd_vel_topic, sonar_topic, touch_topic;
  
  if (!getInput("cmd_vel_topic", cmd_vel_topic)) {
    cmd_vel_topic = "/cmd_vel";
  }
  if (!getInput("sonar_topic", sonar_topic)) {
    sonar_topic = "/sensors/sonar";
  }
  if (!getInput("touch_topic", touch_topic)) {
    touch_topic = "/sensors/touch";
  }
  
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10);
  
  sonar_sub_ = node_->create_subscription<nao_lola_sensor_msgs::msg::Sonar>(
    sonar_topic, 10,
    std::bind(&FollowAction::sonarCallback, this, std::placeholders::_1));
  
  touch_sub_ = node_->create_subscription<nao_lola_sensor_msgs::msg::Touch>(
    touch_topic, 10,
    std::bind(&FollowAction::touchCallback, this, std::placeholders::_1));
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

FollowAction::~FollowAction()
{
  stopRobot();
}

BT::NodeStatus FollowAction::onStart()
{
  if (!getInput("min_distance", min_distance_)) {
    min_distance_ = 1.0;
  }
  if (!getInput("avoidance_distance", avoidance_distance_)) {
    avoidance_distance_ = 0.5;
  }
  if (!getInput("max_linear_speed", max_linear_speed_)) {
    max_linear_speed_ = 0.5;
  }
  if (!getInput("max_angular_speed", max_angular_speed_)) {
    max_angular_speed_ = 1.0;
  }
  if (!getInput("target_frame", target_frame_)) {
    target_frame_ = "target";
  }
  if (!getInput("base_frame", base_frame_)) {
    base_frame_ = "base_link";
  }
  
  vel_rot_avoidance_ = 0.0;
  
  RCLCPP_INFO(node_->get_logger(), 
    "Starting to follow target '%s' (min_dist: %.2f m)",
    target_frame_.c_str(), min_distance_);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowAction::onRunning()
{
  // Process ROS callbacks
  rclcpp::spin_some(node_);
  
  // Check if stopped by touch sensor
  if (stop_requested_) {
    RCLCPP_DEBUG(node_->get_logger(), "Robot stopped by touch sensor");
    stopRobot();
    return BT::NodeStatus::RUNNING;
  }
  
  // Get target position
  try {
    auto transform = tf_buffer_->lookupTransform(
      base_frame_,
      target_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));
    
    double target_x = transform.transform.translation.x;
    double target_y = transform.transform.translation.y;
    
    double distance = std::sqrt(target_x * target_x + target_y * target_y);
    double angle = std::atan2(target_y, target_x);
    
    RCLCPP_DEBUG(node_->get_logger(), 
      "Target at distance: %.2f m, angle: %.2f deg",
      distance, angle * 180.0 / M_PI);
    
    // Calculate velocities
    double vel_lin = 0.0;
    double vel_rot = 0.0;
    
    // Linear velocity - proportional to distance error
    if (distance > min_distance_) {
      vel_lin = std::min(max_linear_speed_, 
                        max_linear_speed_ * (distance - min_distance_));
    }
    
    // Angular velocity - proportional to angle
    vel_rot = std::max(-max_angular_speed_, 
                      std::min(max_angular_speed_, 
                              max_angular_speed_ * angle / (M_PI / 2)));
    
    // Integrate obstacle avoidance
    if (vel_rot * vel_rot_avoidance_ > 0.0) {
      // Same direction, add avoidance
      vel_rot += vel_rot_avoidance_;
      vel_rot = std::max(-max_angular_speed_, 
                        std::min(max_angular_speed_, vel_rot));
    } else if (std::abs(vel_rot_avoidance_) > 0.01) {
      // Override with avoidance if there's an obstacle
      vel_rot = vel_rot_avoidance_;
      vel_lin = vel_lin * 0.5;  // Reduce speed when avoiding
    }
    
    RCLCPP_INFO(node_->get_logger(), 
      "Following: lin=%.2f m/s, ang=%.2f rad/s", vel_lin, vel_rot);
    
    // Publish velocity command
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = vel_lin;
    twist_msg.angular.z = vel_rot;
    cmd_vel_pub_->publish(twist_msg);
    
    return BT::NodeStatus::RUNNING;
    
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Lost target: %s", ex.what());
    stopRobot();
    return BT::NodeStatus::FAILURE;
  }
}

void FollowAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Follow action halted");
  stopRobot();
}

void FollowAction::stopRobot()
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

void FollowAction::sonarCallback(
  const nao_lola_sensor_msgs::msg::Sonar::SharedPtr msg)
{
  double dis_left = msg->left;
  double dis_right = msg->right;
  
  bool avoid_left = false;
  bool avoid_right = false;
  double vel_rot_avoid_left = 0.0;
  double vel_rot_avoid_right = 0.0;
  
  vel_rot_avoidance_ = 0.0;
  
  if (dis_left < avoidance_distance_) {
    RCLCPP_DEBUG(node_->get_logger(), 
      "Obstacle detected on LEFT at %.2f m", dis_left);
    avoid_left = true;
    vel_rot_avoid_left = -max_angular_speed_;
    vel_rot_avoidance_ = vel_rot_avoid_left;
  }
  
  if (dis_right < avoidance_distance_) {
    RCLCPP_DEBUG(node_->get_logger(), 
      "Obstacle detected on RIGHT at %.2f m", dis_right);
    avoid_right = true;
    vel_rot_avoid_right = max_angular_speed_;
    vel_rot_avoidance_ = vel_rot_avoid_right;
  }
  
  if (avoid_left && avoid_right) {
    // Both sides blocked, turn away from closer obstacle
    if (dis_left < dis_right) {
      vel_rot_avoidance_ = vel_rot_avoid_left;
    } else if (dis_right < dis_left) {
      vel_rot_avoidance_ = vel_rot_avoid_right;
    } else {
      vel_rot_avoidance_ = 0.0;
    }
  }
}

void FollowAction::touchCallback(
  const nao_lola_sensor_msgs::msg::Touch::SharedPtr msg)
{
  if (msg->head_front || msg->head_middle || msg->head_rear) {
    stop_requested_ = !stop_requested_;
    if (stop_requested_) {
      RCLCPP_INFO(node_->get_logger(), "Touch detected: Stopping robot");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Touch detected: Resuming robot");
    }
  }
}

}  // namespace follow_nao
