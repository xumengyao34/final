#ifndef MBF_NAV2_CONNECTOR__NAV2_CONTROLLER_HPP_
#define MBF_NAV2_CONNECTOR__NAV2_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mbf_simple_core/simple_controller.h"
#include "mbf_msgs/action/exe_path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace mbf_nav2_connector
{

/**
 * @brief Nav2 Controller Plugin that forwards MBF controller calls to Nav2 FollowPath action
 * 
 * This plugin implements the MBF SimpleController interface and acts as a client
 * to Nav2's FollowPath action server, effectively bridging MBF and Nav2.
 */
class Nav2Controller : public mbf_simple_core::SimpleController
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  /**
   * @brief Constructor
   */
  Nav2Controller();

  /**
   * @brief Destructor
   */
  virtual ~Nav2Controller();

  /**
   * @brief Initialize the controller plugin
   * @param name The name of the controller
   * @param tf Shared pointer to a TF buffer
   * @param node_handle Shared pointer to the ROS 2 node
   */
  void initialize(const std::string name, const std::shared_ptr<tf2_ros::Buffer>& tf,
                  const rclcpp::Node::SharedPtr& node_handle) override;

  /**
   * @brief Set the plan to be executed
   * @param plan The plan to follow
   * @return True if the plan was set successfully
   */
  bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan) override;

  /**
   * @brief Compute velocity commands to follow the plan
   * @param pose The current robot pose
   * @param velocity The current robot velocity
   * @param cmd_vel The computed velocity command
   * @param message An optional message about the control result
   * @return The outcome code indicating success, failure, or goal reached
   */
  uint32_t computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::TwistStamped& velocity,
    geometry_msgs::msg::TwistStamped& cmd_vel,
    std::string& message) override;

  /**
   * @brief Check if the goal has been reached
   * @param xy_tolerance XY distance tolerance
   * @param yaw_tolerance Yaw angle tolerance
   * @return True if the goal has been reached within tolerances
   */
  bool isGoalReached(double xy_tolerance, double yaw_tolerance) override;

  /**
   * @brief Cancel the current control operation
   * @return True if cancellation was successful
   */
  bool cancel() override;

private:
  /**
   * @brief Callback for goal response
   */
  void goalResponseCallback(const GoalHandleFollowPath::SharedPtr& goal_handle);

  /**
   * @brief Callback for feedback
   */
  void feedbackCallback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback);

  /**
   * @brief Callback for result
   */
  void resultCallback(const GoalHandleFollowPath::WrappedResult& result);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  std::string name_;
  
  // Action state
  GoalHandleFollowPath::SharedPtr goal_handle_;
  std::atomic<bool> action_active_{false};
  std::atomic<bool> action_completed_{false};
  std::atomic<bool> action_succeeded_{false};
  std::string result_message_;
  
  // Feedback state
  std::mutex feedback_mutex_;
  double distance_to_goal_{std::numeric_limits<double>::max()};
  geometry_msgs::msg::TwistStamped current_cmd_vel_;
  
  // Parameters
  std::string nav2_controller_action_;
  std::string nav2_controller_name_;
  double action_timeout_;
  bool goal_reached_{false};
};

}  // namespace mbf_nav2_connector

#endif  // MBF_NAV2_CONNECTOR__NAV2_CONTROLLER_HPP_
