#include "navigation_behaviors.h"
#include "yaml-cpp/yaml.h"
#include <string>

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr), done_flag_(false), has_received_goal_(false)
{
    // Create the subscription to the /book_goal_point topic with PointStamped type
    subscription_ = node_ptr_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/book_goal_point", rclcpp::QoS(10), 
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            handleGoalMessage(msg);  // Call to the member function
        });

    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
}

BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
    // If a goal has not been received yet, we will not do anything here.
    return BT::NodeStatus::RUNNING; 
}

// Method to check if goal has been reached 
BT::NodeStatus GoToPose::onRunning()
{
    if (done_flag_)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    // If there is a result, we consider navigation completed.
    if (result.result)
    {
        done_flag_ = true;
    }
}

void GoToPose::handleGoalMessage(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (msg) { // Check if the msg is not null
        // Convert PointStamped to PoseStamped
        goal_pose_.header = msg->header; // Copy header from PointStamped
        goal_pose_.pose.position = msg->point; // Use the point as position
        goal_pose_.pose.orientation.w = 1.0; // Set a default orientation (no rotation)

        has_received_goal_ = true; // Set the flag to true
        
        // Print the received goal details
        RCLCPP_INFO(node_ptr_->get_logger(), "Received goal point: x=%.2f, y=%.2f, z=%.2f", 
                    msg->point.x, msg->point.y, msg->point.z);
        
        // Now check if the action server is available before sending the goal
        if (!action_client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_ptr_->get_logger(), "Action server not available, cannot send goal.");
            return; // Exit if the action server is not available
        }

        // Send the goal to the action server
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose_; // Use the goal pose received from the topic

        done_flag_ = false;
        action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
    } else {
        RCLCPP_WARN(node_ptr_->get_logger(), "Received null goal point");
    }
}
