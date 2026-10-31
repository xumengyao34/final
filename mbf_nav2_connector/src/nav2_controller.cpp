#include "mbf_nav2_connector/nav2_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace mbf_nav2_connector
{

Nav2Controller::Nav2Controller()
  : nav2_controller_action_("follow_path"),
    nav2_controller_name_("FollowPath"),
    action_timeout_(60.0)
{
}

Nav2Controller::~Nav2Controller()
{
  if (action_active_) {
    cancel();
  }
}

void Nav2Controller::initialize(const std::string name, const std::shared_ptr<tf2_ros::Buffer>& tf,
                                const rclcpp::Node::SharedPtr& node_handle)
{
  name_ = name;
  node_ = node_handle;
  
  // Declare and get parameters
  node_->declare_parameter(name_ + ".nav2_controller_action", nav2_controller_action_);
  node_->declare_parameter(name_ + ".nav2_controller_name", nav2_controller_name_);
  node_->declare_parameter(name_ + ".action_timeout", action_timeout_);
  
  node_->get_parameter(name_ + ".nav2_controller_action", nav2_controller_action_);
  node_->get_parameter(name_ + ".nav2_controller_name", nav2_controller_name_);
  node_->get_parameter(name_ + ".action_timeout", action_timeout_);
  
  // Create action client
  action_client_ = rclcpp_action::create_client<FollowPath>(
    node_,
    nav2_controller_action_);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Nav2Controller plugin '%s' initialized. Connecting to action server '%s' with controller '%s'",
    name_.c_str(), nav2_controller_action_.c_str(), nav2_controller_name_.c_str());
  
  // Wait for action server
  if (!action_client_->wait_for_action_server(10s)) {
    RCLCPP_WARN(node_->get_logger(),
      "Action server '%s' not available after waiting. Will retry on first control request.",
      nav2_controller_action_.c_str());
  }
}

bool Nav2Controller::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  // Cancel any existing action
  if (action_active_) {
    RCLCPP_INFO(node_->get_logger(), "Canceling previous path following action");
    cancel();
  }
  
  // Wait for action server if not available
  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Action server %s not available", nav2_controller_action_.c_str());
    return false;
  }
  
  if (plan.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot set empty plan");
    return false;
  }
  
  // Create goal message
  auto goal_msg = FollowPath::Goal();
  goal_msg.path.header = plan.front().header;
  goal_msg.path.poses = plan;
  
  // Set controller_id if specified (tells Nav2 which controller plugin to use)
  if (!nav2_controller_name_.empty()) {
    goal_msg.controller_id = nav2_controller_name_;
    RCLCPP_INFO(node_->get_logger(), "Forwarding to Nav2 controller: %s", nav2_controller_name_.c_str());
  }
  
  // Reset state
  action_active_ = false;
  action_completed_ = false;
  action_succeeded_ = false;
  result_message_ = "";
  goal_handle_ = nullptr;
  goal_reached_ = false;
  distance_to_goal_ = std::numeric_limits<double>::max();
  
  // Send goal
  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Nav2Controller::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&Nav2Controller::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&Nav2Controller::resultCallback, this, std::placeholders::_1);
  
  RCLCPP_INFO(node_->get_logger(), "Sending path with %zu poses to Nav2 controller", plan.size());
  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
  
  // Wait for goal to be accepted (don't spin - MBF already has an executor)
  if (goal_handle_future.wait_for(5s) != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to action server - timeout");
    return false;
  }
  
  goal_handle_ = goal_handle_future.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return false;
  }
  
  action_active_ = true;
  RCLCPP_INFO(node_->get_logger(), "Path following action started");
  return true;
}

uint32_t Nav2Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::TwistStamped& velocity,
  geometry_msgs::msg::TwistStamped& cmd_vel,
  std::string& message)
{
  // MBF's executor will process callbacks automatically
  // This function is called at controller_frequency (default 20Hz)
  // Since Nav2 handles velocity commands internally, we just monitor status
  
  // Check if action is still active
  if (!action_active_) {
    message = "No active path following action";
    RCLCPP_DEBUG(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::ExePath::Result::INTERNAL_ERROR;
  }
  
  // Check if action has completed
  if (action_completed_) {
    if (action_succeeded_) {
      message = "Goal reached";
      RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
      goal_reached_ = true;
      
      // Return zero velocity
      cmd_vel = geometry_msgs::msg::TwistStamped();
      cmd_vel.header.stamp = node_->now();
      cmd_vel.header.frame_id = pose.header.frame_id;
      
      return mbf_msgs::action::ExePath::Result::SUCCESS;
    } else {
      message = result_message_.empty() ? "Path following failed" : result_message_;
      RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
      return mbf_msgs::action::ExePath::Result::FAILURE;
    }
  }
  
  // Get current velocity command from feedback
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    cmd_vel = current_cmd_vel_;
  }
  
  // Ensure the velocity has proper timestamp and frame
  cmd_vel.header.stamp = node_->now();
  if (cmd_vel.header.frame_id.empty()) {
    cmd_vel.header.frame_id = "base_link";
  }
  
  message = "Following path, distance to goal: " + std::to_string(distance_to_goal_);
  return mbf_msgs::action::ExePath::Result::SUCCESS;
}

bool Nav2Controller::isGoalReached(double xy_tolerance, double yaw_tolerance)
{
  // Check if goal was marked as reached
  if (goal_reached_) {
    return true;
  }
  
  // Check if action succeeded
  if (action_completed_ && action_succeeded_) {
    goal_reached_ = true;
    return true;
  }
  
  // Check distance to goal from feedback
  if (distance_to_goal_ < xy_tolerance) {
    // Note: Nav2 FollowPath doesn't provide yaw difference in feedback
    // We trust that Nav2's controller has reached the goal if action succeeded
    return action_completed_ && action_succeeded_;
  }
  
  return false;
}

bool Nav2Controller::cancel()
{
  if (goal_handle_ && action_active_) {
    RCLCPP_INFO(node_->get_logger(), "Canceling path following action");
    auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    
    if (cancel_future.wait_for(2s) == std::future_status::ready)
    {
      RCLCPP_INFO(node_->get_logger(), "Path following action canceled successfully");
      action_active_ = false;
      action_completed_ = true;
      action_succeeded_ = false;
      return true;
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "Failed to cancel path following action");
      return false;
    }
  }
  return true;
}

void Nav2Controller::goalResponseCallback(const GoalHandleFollowPath::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    action_active_ = false;
    action_completed_ = true;
    action_succeeded_ = false;
    result_message_ = "Goal rejected by action server";
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server");
  }
}

void Nav2Controller::feedbackCallback(
  GoalHandleFollowPath::SharedPtr,
  const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  
  // Update distance to goal
  distance_to_goal_ = feedback->distance_to_goal;
  
  // Store current velocity command
  // Note: Nav2's FollowPath feedback doesn't include velocity, so we can't update it here
  // The actual velocity commands are sent directly by Nav2's controller
  // For MBF compatibility, we'll return zero velocity in computeVelocityCommands
  // since Nav2 handles the velocity commands internally
  
  RCLCPP_DEBUG(node_->get_logger(), 
    "Path following feedback - Distance to goal: %.2f m", distance_to_goal_);
}

void Nav2Controller::resultCallback(const GoalHandleFollowPath::WrappedResult& result)
{
  action_completed_ = true;
  action_active_ = false;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      action_succeeded_ = true;
      result_message_ = "Path following succeeded";
      goal_reached_ = true;
      RCLCPP_INFO(node_->get_logger(), "%s", result_message_.c_str());
      break;
      
    case rclcpp_action::ResultCode::ABORTED:
      action_succeeded_ = false;
      result_message_ = "Path following was aborted";
      RCLCPP_ERROR(node_->get_logger(), "%s", result_message_.c_str());
      break;
      
    case rclcpp_action::ResultCode::CANCELED:
      action_succeeded_ = false;
      result_message_ = "Path following was canceled";
      RCLCPP_WARN(node_->get_logger(), "%s", result_message_.c_str());
      break;
      
    default:
      action_succeeded_ = false;
      result_message_ = "Unknown result code";
      RCLCPP_ERROR(node_->get_logger(), "%s", result_message_.c_str());
      break;
  }
}

}  // namespace mbf_nav2_connector

// Register this controller as a mbf_simple_core::SimpleController plugin
PLUGINLIB_EXPORT_CLASS(mbf_nav2_connector::Nav2Controller, mbf_simple_core::SimpleController)
