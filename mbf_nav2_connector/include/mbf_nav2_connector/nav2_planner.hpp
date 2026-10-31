#ifndef MBF_NAV2_CONNECTOR__NAV2_PLANNER_HPP_
#define MBF_NAV2_CONNECTOR__NAV2_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mbf_simple_core/simple_planner.h"
#include "mbf_msgs/action/get_path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace mbf_nav2_connector
{

/**
 * @brief Nav2 Planner Plugin that forwards MBF planner calls to Nav2 ComputePathToPose action
 * 
 * This plugin implements the MBF SimplePlanner interface and acts as a client
 * to Nav2's ComputePathToPose action server, effectively bridging MBF and Nav2.
 */
class Nav2Planner : public mbf_simple_core::SimplePlanner
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  /**
   * @brief Constructor
   */
  Nav2Planner();

  /**
   * @brief Destructor
   */
  virtual ~Nav2Planner();

  /**
   * @brief Initialize the planner plugin
   * @param name The name of the planner
   * @param node_handle Shared pointer to the ROS 2 node
   */
  void initialize(const std::string name, const rclcpp::Node::SharedPtr& node_handle) override;

  /**
   * @brief Make a plan from start to goal pose
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance The tolerance for the goal
   * @param plan The computed plan
   * @param cost The cost of the plan
   * @param message An optional message about the planning result
   * @return The outcome code indicating success or failure
   */
  uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    double tolerance,
    std::vector<geometry_msgs::msg::PoseStamped>& plan,
    double& cost,
    std::string& message) override;

  /**
   * @brief Cancel the current planning operation
   * @return True if cancellation was successful
   */
  bool cancel() override;

private:
  /**
   * @brief Callback for goal response
   */
  void goalResponseCallback(const GoalHandleComputePath::SharedPtr& goal_handle);

  /**
   * @brief Callback for feedback
   */
  void feedbackCallback(
    GoalHandleComputePath::SharedPtr,
    const std::shared_ptr<const ComputePathToPose::Feedback> feedback);

  /**
   * @brief Callback for result
   */
  void resultCallback(const GoalHandleComputePath::WrappedResult& result);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
  std::string name_;
  
  // Action state
  GoalHandleComputePath::SharedPtr goal_handle_;
  std::atomic<bool> action_completed_{false};
  std::atomic<bool> action_succeeded_{false};
  nav_msgs::msg::Path result_path_;
  std::string result_message_;
  
  // Parameters
  std::string nav2_planner_action_;
  std::string nav2_planner_name_;
  std::string global_frame_;
  double action_timeout_;
};

}  // namespace mbf_nav2_connector

#endif  // MBF_NAV2_CONNECTOR__NAV2_PLANNER_HPP_
