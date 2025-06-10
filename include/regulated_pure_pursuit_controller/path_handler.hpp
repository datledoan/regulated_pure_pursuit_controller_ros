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

#ifndef REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
#define REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/odometry_helper_ros.h>
#include "regulated_pure_pursuit_controller/geometry_utils.h"
#include <geometry_msgs/Pose2D.h>

namespace regulated_pure_pursuit_controller
{

/**
 * @class regulated_pure_pursuit_controller::PathHandler
 * @brief Handles input paths to transform them to local frames required
 */
class PathHandler
{
public:
  /**
   * @brief Constructor for regulated_pure_pursuit_controller::PathHandler
   */
  PathHandler(
    ros::Duration transform_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Destrructor for regulated_pure_pursuit_controller::PathHandler
   */
  ~PathHandler();

  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @param max_robot_pose_search_dist Distance to search for matching nearest path point
   * @param reject_unit_path If true, fail if path has only one pose
   * @return Path in new frame
   */
  nav_msgs::Path transformGlobalPlan(
    const geometry_msgs::PoseStamped & pose,
    double max_robot_pose_search_dist, bool reject_unit_path = false);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame,
    const geometry_msgs::PoseStamped & in_pose,
    geometry_msgs::PoseStamped & out_pose) const;

  void setPlan(const nav_msgs::Path & path) {global_plan_ = path;}

  nav_msgs::Path getPlan() {return global_plan_;}

protected:
  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  ros::Duration transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::Path global_plan_;
};

}  // namespace regulated_pure_pursuit_controller

#endif  // REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
