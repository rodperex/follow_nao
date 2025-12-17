#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "follow_nao/bt_nodes/is_target_detected_condition.hpp"
#include "follow_nao/bt_nodes/spin_search_action.hpp"
#include "follow_nao/bt_nodes/follow_action.hpp"

class BehaviorTreeFollowNode : public rclcpp::Node
{
public:
  BehaviorTreeFollowNode()
  : Node("bt_follow_node")
  {
    this->declare_parameter("bt_xml", "");
    this->declare_parameter("tick_rate", 10.0);
    
    std::string bt_xml_file = this->get_parameter("bt_xml").as_string();
    double tick_rate = this->get_parameter("tick_rate").as_double();
    
    if (bt_xml_file.empty()) {
      RCLCPP_ERROR(this->get_logger(), 
        "Parameter 'bt_xml' not set. Please provide the path to the behavior tree XML file.");
      rclcpp::shutdown();
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loading behavior tree from: %s", bt_xml_file.c_str());
    
    // Create BT factory and register custom nodes
    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<follow_nao::IsTargetDetectedCondition>("IsTargetDetected");
    factory.registerNodeType<follow_nao::SpinSearchAction>("SpinSearch");
    factory.registerNodeType<follow_nao::FollowAction>("Follow");
    
    try {
      // Create tree from XML
      tree_ = factory.createTreeFromFile(bt_xml_file);
      
      RCLCPP_INFO(this->get_logger(), "Behavior tree loaded successfully");
      
      // Optional: Create a logger for the tree
      // logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
      
      // Create timer to tick the tree
      auto period = std::chrono::duration<double>(1.0 / tick_rate);
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&BehaviorTreeFollowNode::tickTree, this));
      
      RCLCPP_INFO(this->get_logger(), 
        "Behavior tree executor started (tick rate: %.1f Hz)", tick_rate);
      
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), 
        "Failed to create behavior tree: %s", e.what());
      rclcpp::shutdown();
    }
  }

private:
  void tickTree()
  {
    if (tree_.rootNode()) {
      auto status = tree_.tickOnce();
      
      switch (status) {
        case BT::NodeStatus::SUCCESS:
          RCLCPP_INFO_ONCE(this->get_logger(), "Behavior tree completed successfully");
          break;
        case BT::NodeStatus::FAILURE:
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Behavior tree returned FAILURE");
          break;
        case BT::NodeStatus::RUNNING:
          RCLCPP_DEBUG(this->get_logger(), "Behavior tree running...");
          break;
        default:
          break;
      }
    }
  }

  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  // std::unique_ptr<BT::StdCoutLogger> logger_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorTreeFollowNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
