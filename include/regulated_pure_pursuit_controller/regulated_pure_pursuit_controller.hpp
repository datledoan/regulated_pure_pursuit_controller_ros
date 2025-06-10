// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/Bool.h"
#include <nav_core/base_local_planner.h>
#include "regulated_pure_pursuit_controller/path_handler.hpp"
#include "regulated_pure_pursuit_controller/collision_checker.hpp"
#include "regulated_pure_pursuit_controller/parameter_handler.hpp"
#include "regulated_pure_pursuit_controller/regulation_functions.hpp"
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/costmap_model.h>

#include <mbf_costmap_core/costmap_controller.h>
#include <mbf_msgs/ExePathResult.h>

namespace regulated_pure_pursuit_controller
{

/**
 * @class regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public nav_core::BaseLocalPlanner, public mbf_costmap_core::CostmapController
{
public:
  /**
   * @brief Constructor for regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  RegulatedPurePursuitController() = default;

  /**
   * @brief Destrructor for regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  ~RegulatedPurePursuitController() = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void initialize(
    std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief move_base_flex api compute the best command given the current pose and velocity, with possible debug information
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @param cmd_vel   Best command
   * @param message   Debug information
   * @return          move_base_flex result code
   */
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                  const geometry_msgs::TwistStamped& velocity,
                                  geometry_msgs::TwistStamped& cmd_vel,
                                  std::string& message);
  
  /**
   * @brief move_base api compute the best command given the current pose and velocity
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param cmd_vel   Best command
   * @return          true if a valid command was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool cancel() { 
      return false; 
  };

  /**
   * @brief Sets the global plan
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

protected:
  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::Twist &);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  geometry_msgs::PointStamped createCarrotMsg(
    const geometry_msgs::PoseStamped & carrot_pose);

  /**
   * @brief Whether robot should rotate to rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relative to carrot marker
   * @param x_vel_sign Velocoty sign (forward or backward)
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path,
    double & x_vel_sign);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relative to carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::Twist & curr_speed);

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed Speed of robot
   * @param pose_cost cost at this pose
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::Twist & speed,
    const double & pose_cost, const nav_msgs::Path & path,
    double & linear_vel, double & sign);

  /**
   * @brief Find the intersection a circle and a line segment.
   * This assumes the circle is centered at the origin.
   * If no intersection is found, a floating point error will occur.
   * @param p1 first endpoint of line segment
   * @param p2 second endpoint of line segment
   * @param r radius of circle
   * @return point of intersection
   */
  static geometry_msgs::Point circleSegmentIntersection(
    const geometry_msgs::Point & p1,
    const geometry_msgs::Point & p2,
    double r);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @param interpolate_after_goal If true, interpolate the lookahead point after the goal based
   * on the orientation given by the position of the last two pose of the path
   * @return Lookahead point
   */
  geometry_msgs::PoseStamped getLookAheadPoint(
    const double &, const nav_msgs::Path &,
    bool interpolate_after_goal = false);

  /**
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double findVelocitySignChange(const nav_msgs::Path & transformed_plan);

  void getRobotVel(geometry_msgs::Twist& speed);

  void createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path);

  /**
   * @brief move_base api whether the goal has been reached
   * @return Whether the goal has been reached
   */
  bool isGoalReached();

  /**
   * @brief move_base_flex api whether the goal has been reached
   * @return Whether the goal has been reached
   */
  bool isGoalReached(double xy_tolerance, double yaw_tolerance); 
  bool isThetaGoalReached(double dtheta, double angle_tolerance, double max_angular_vel, double dt);

  const bool& isInitialized()
  {
    return initialized_;
  }

  /**
   * @brief Calculate the turning radius of the robot given a target point
   * @param target_pose The target pose to calculate the turning radius
   * @return The turning radius of the robot
   */
  double calcTurningRadius(const geometry_msgs::PoseStamped & target_pose);

  /**
   * @brief Get the orientation of a vector
   * @param p1 The first point of the vector
   * @param p2 The second point of the vector
   * @return The orientation of the vector
   */
  geometry_msgs::Quaternion getOrientation(
    const geometry_msgs::Point & p1,
    const geometry_msgs::Point & p2);

  std::shared_ptr<ros::NodeHandle> node_;
  std::shared_ptr<ros::NodeHandle> private_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  costmap_2d::Costmap2D * costmap_;
  base_local_planner::OdometryHelperRos odom_helper_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  Parameters * params_;
  double goal_dist_tol_;
  double angle_tolerance_;
  double max_angular_vel_;
  double theta_stopped_vel_;
  double trans_stopped_vel_;
  double control_duration_;
  bool is_rotating_to_heading_ = false;
  bool goal_reached_ = false;
  bool initialized_ = false;

  ros::Publisher global_path_pub_;
  ros::Publisher global_path_origin_pub_;
  ros::Publisher carrot_pub_;
  ros::Publisher vector_pub_;
  ros::Publisher curvature_carrot_pub_;
  ros::Publisher carrot_arc_pub_;
  std::unique_ptr<regulated_pure_pursuit_controller::PathHandler> path_handler_;
  std::unique_ptr<regulated_pure_pursuit_controller::ParameterHandler> param_handler_;
  std::unique_ptr<regulated_pure_pursuit_controller::CollisionChecker> collision_checker_;
  std::string odom_topic_{"odom"};
};

}  // namespace regulated_pure_pursuit_controller

#endif  // REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
