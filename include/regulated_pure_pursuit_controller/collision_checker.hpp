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

#ifndef REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_
#define REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/Pose2D.h>
#include "regulated_pure_pursuit_controller/parameter_handler.hpp"
#include "regulated_pure_pursuit_controller/geometry_utils.h"

namespace regulated_pure_pursuit_controller
{

/**
 * @class regulated_pure_pursuit_controller::CollisionChecker
 * @brief Checks for collision based on a RPP control command
 */
class CollisionChecker
{
public:
  /**
   * @brief Constructor for regulated_pure_pursuit_controller::CollisionChecker
   */
  CollisionChecker(
    std::shared_ptr<ros::NodeHandle> node,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros, Parameters * params);

  /**
   * @brief Destrructor for regulated_pure_pursuit_controller::CollisionChecker
   */
  ~CollisionChecker() = default;

  /**
   * @brief Whether collision is imminent
   * @param robot_pose Pose of robot
   * @param carrot_pose Pose of carrot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @param carrot_dist Distance to the carrot for PP
   * @return Whether collision is imminent
   */
  bool isCollisionImminent(
    const geometry_msgs::PoseStamped &,
    const double &, const double &,
    const double &);

  /**
   * @brief checks for collision at projected pose
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @param theta orientation of Yaw
   * @return Whether in collision
   */
  bool inCollision(
    const double & x,
    const double & y,
    const double & theta);

  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in costmap
   */
  double costAtPose(const double & x, const double & y);

protected:
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  base_local_planner::CostmapModel* footprint_collision_checker_;
  Parameters * params_;
  ros::Publisher carrot_arc_pub_;
};

}  // namespace regulated_pure_pursuit_controller

#endif  // REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_
