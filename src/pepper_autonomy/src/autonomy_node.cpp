#include "autonomy_node.h"


using namespace std::chrono_literals;

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("pepper_autonomy") + "/bt_xml";

AutonomyNode::AutonomyNode(const std::string &nodeName) : Node(nodeName)
{
  this->declare_parameter("location_file","none");

  RCLCPP_INFO(get_logger(), "Init done");
}

void AutonomyNode::setup()
{
  RCLCPP_INFO(get_logger(), "Setting up");
  create_behavior_tree();
  RCLCPP_INFO(get_logger(), "BT created");

  const auto timer_period = 500ms;
  timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&AutonomyNode::update_behavior_tree, this));

  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
}

void AutonomyNode::create_behavior_tree()
{
  BT::BehaviorTreeFactory factory;

  // register bt node

  BT::NodeBuilder builder =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<GoToPose>(name, config, shared_from_this());
  };

  factory.registerBuilder<GoToPose>("GoToPose", builder);

  RCLCPP_INFO(get_logger(), bt_xml_dir.c_str());

  tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
  RCLCPP_INFO(get_logger(), "3");
}

void AutonomyNode::update_behavior_tree()
{
    BT::NodeStatus tree_status = tree_.tickRoot();

    if (tree_status == BT::NodeStatus::RUNNING)
    {
        return; // Keep running the behavior tree
    }
    else if (tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Finished Navigation");
        timer_->cancel(); // Stop the timer if navigation finished successfully
    }
    else if (tree_status == BT::NodeStatus::FAILURE)
    {
        // Check if the failure is due to lack of goal pose
        // You might want to log if a valid goal was never received
        RCLCPP_WARN(this->get_logger(), "Navigation Failed due to invalid goal pose or other reason");

        // Optionally, implement a check to retry or wait for a goal
        // Here you can restart the behavior tree, or reset some states if needed
        timer_->cancel();
    }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomyNode>("autonomy_node");
  node->setup();

  return 0;
}