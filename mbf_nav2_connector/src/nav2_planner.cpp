#include "mbf_nav2_connector/nav2_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace mbf_nav2_connector
{

Nav2Planner::Nav2Planner()
  : nav2_planner_action_("compute_path_to_pose"),
    nav2_planner_name_("GridBased"),
    action_timeout_(30.0)
{
}

Nav2Planner::~Nav2Planner()
{
}

void Nav2Planner::initialize(const std::string name, const rclcpp::Node::SharedPtr& node_handle)
{
  name_ = name;
  node_ = node_handle;
  
  // Declare and get parameters
  node_->declare_parameter(name_ + ".nav2_planner_action", nav2_planner_action_);
  node_->declare_parameter(name_ + ".nav2_planner_name", nav2_planner_name_);
  node_->declare_parameter(name_ + ".action_timeout", action_timeout_);
  // [2026-10-27] nav2 返回的 path 不带 frame_id (空字符串), mbf get_path 层
  // 要把 plan 转到 global frame 时直接报 "Invalid frame ID ''"; 这里补齐
  node_->declare_parameter(name_ + ".global_frame", std::string("map"));

  node_->get_parameter(name_ + ".nav2_planner_action", nav2_planner_action_);
  node_->get_parameter(name_ + ".nav2_planner_name", nav2_planner_name_);
  node_->get_parameter(name_ + ".action_timeout", action_timeout_);
  node_->get_parameter(name_ + ".global_frame", global_frame_);
  
  // Create action client
  action_client_ = rclcpp_action::create_client<ComputePathToPose>(
    node_,
    nav2_planner_action_);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Nav2Planner plugin '%s' initialized. Connecting to action server '%s' with planner '%s'",
    name_.c_str(), nav2_planner_action_.c_str(), nav2_planner_name_.c_str());
  
  // Wait for action server
  if (!action_client_->wait_for_action_server(10s)) {
    RCLCPP_WARN(node_->get_logger(),
      "Action server '%s' not available after waiting. Will retry on first plan request.",
      nav2_planner_action_.c_str());
  }
}

uint32_t Nav2Planner::makePlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  double tolerance,
  std::vector<geometry_msgs::msg::PoseStamped>& plan,
  double& cost,
  std::string& message)
{
  RCLCPP_INFO(node_->get_logger(), "[makePlan] Starting planning request");
  
  // Wait for action server if not available
  if (!action_client_->wait_for_action_server(5s)) {
    message = "Action server " + nav2_planner_action_ + " not available";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
  }
  
  RCLCPP_INFO(node_->get_logger(), "[makePlan] Action server available");
  
  try {
  
  // Create goal message
  auto goal_msg = ComputePathToPose::Goal();
  goal_msg.goal = goal;
  goal_msg.start = start;
  goal_msg.use_start = true;
  
  // Set planner_id if specified (tells Nav2 which planner plugin to use)
  if (!nav2_planner_name_.empty()) {
    goal_msg.planner_id = nav2_planner_name_;
    RCLCPP_INFO(node_->get_logger(), "Forwarding to Nav2 planner: %s", nav2_planner_name_.c_str());
  }
  
  // If planner supports tolerance, we could add it here
  // Nav2 doesn't have a direct tolerance parameter in ComputePathToPose

  // Reset state
  action_completed_ = false;
  action_succeeded_ = false;
  result_path_ = nav_msgs::msg::Path();
  result_message_ = "";
  goal_handle_ = nullptr;
  
  // Send goal
  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Nav2Planner::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&Nav2Planner::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&Nav2Planner::resultCallback, this, std::placeholders::_1);
  
  RCLCPP_INFO(node_->get_logger(), "[makePlan] Sending planning request to Nav2");
  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_->get_logger(), "[makePlan] async_send_goal called, waiting for response");
  
  // Wait for goal to be accepted (don't spin - MBF already has an executor)
  auto status = goal_handle_future.wait_for(5s);
  if (status != std::future_status::ready)
  {
    message = "Failed to send goal to action server - timeout";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
  }
  
  RCLCPP_INFO(node_->get_logger(), "[makePlan] Getting goal handle from future");
  goal_handle_ = goal_handle_future.get();
  RCLCPP_INFO(node_->get_logger(), "[makePlan] Got goal handle, checking if valid");
  
  if (!goal_handle_) {
    message = "Goal was rejected by server";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), "[makePlan] Goal handle valid, waiting for result");
  
  // Wait for result with timeout (MBF's executor will process callbacks)
  auto timeout = std::chrono::duration<double>(action_timeout_);
  auto start_time = std::chrono::steady_clock::now();
  
  while (!action_completed_ && 
         (std::chrono::steady_clock::now() - start_time) < timeout)
  {
    std::this_thread::sleep_for(10ms);
  }
  
  if (!action_completed_) {
    message = "Planning action timed out after " + std::to_string(action_timeout_) + " seconds";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    cancel();
    return mbf_msgs::action::GetPath::Result::CANCELED;
  }
  
  if (!action_succeeded_) {
    message = result_message_.empty() ? "Planning failed" : result_message_;
    RCLCPP_ERROR(node_->get_logger(), "Planning failed: %s", message.c_str());
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }
  
  // Convert path to plan; 补上 frame_id (nav2 的 path 不带, mbf 转换会报空帧)
  plan.clear();
  plan.reserve(result_path_.poses.size());
  for (auto pose : result_path_.poses) {
    pose.header.frame_id = global_frame_;
    pose.header.stamp = node_->get_clock()->now();
    plan.push_back(pose);
  }
  
  // Calculate approximate cost (path length)
  cost = 0.0;
  for (size_t i = 1; i < plan.size(); ++i) {
    double dx = plan[i].pose.position.x - plan[i-1].pose.position.x;
    double dy = plan[i].pose.position.y - plan[i-1].pose.position.y;
    cost += std::sqrt(dx*dx + dy*dy);
  }
  
  message = "Planning succeeded with path length: " + std::to_string(cost);
  RCLCPP_INFO(node_->get_logger(), "[makePlan] %s (plan size: %zu)", message.c_str(), plan.size());
  
  return mbf_msgs::action::GetPath::Result::SUCCESS;
  
  } catch (const std::exception& e) {
    message = std::string("Exception in makePlan: ") + e.what();
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
  } catch (...) {
    message = "Unknown exception in makePlan";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
  }
}

bool Nav2Planner::cancel()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "Canceling planning action");
    auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    
    if (cancel_future.wait_for(2s) == std::future_status::ready)
    {
      RCLCPP_INFO(node_->get_logger(), "Planning action canceled successfully");
      return true;
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "Failed to cancel planning action");
      return false;
    }
  }
  return true;
}

void Nav2Planner::goalResponseCallback(const GoalHandleComputePath::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    action_completed_ = true;
    action_succeeded_ = false;
    result_message_ = "Goal rejected by action server";
  } else {
    goal_handle_ = goal_handle;
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server");
  }
}

void Nav2Planner::feedbackCallback(
  GoalHandleComputePath::SharedPtr,
  const std::shared_ptr<const ComputePathToPose::Feedback> feedback)
{
  // Log feedback if needed
  RCLCPP_DEBUG(node_->get_logger(), "Received feedback from planner");
}

void Nav2Planner::resultCallback(const GoalHandleComputePath::WrappedResult& result)
{
  action_completed_ = true;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      action_succeeded_ = true;
      result_path_ = result.result->path;
      result_message_ = "Planning succeeded";
      RCLCPP_INFO(node_->get_logger(), "Planning succeeded with %zu poses", 
                  result_path_.poses.size());
      break;
      
    case rclcpp_action::ResultCode::ABORTED:
      action_succeeded_ = false;
      result_message_ = "Planning was aborted";
      RCLCPP_ERROR(node_->get_logger(), "%s", result_message_.c_str());
      break;
      
    case rclcpp_action::ResultCode::CANCELED:
      action_succeeded_ = false;
      result_message_ = "Planning was canceled";
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

// Register this planner as a mbf_simple_core::SimplePlanner plugin
PLUGINLIB_EXPORT_CLASS(mbf_nav2_connector::Nav2Planner, mbf_simple_core::SimplePlanner)
