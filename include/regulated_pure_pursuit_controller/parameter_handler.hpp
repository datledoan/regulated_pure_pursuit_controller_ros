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

#ifndef REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_
#define REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include "regulated_pure_pursuit_controller/geometry_utils.h"
#include <dynamic_reconfigure/server.h>
#include "regulated_pure_pursuit_controller/RegulatedPurePursuitConfig.h"

namespace regulated_pure_pursuit_controller
{

struct Parameters
{
  double desired_linear_vel;
  double lookahead_dist;
  double rotate_to_heading_angular_vel;
  double max_lookahead_dist;
  double min_lookahead_dist;
  double lookahead_time;
  bool use_velocity_scaled_lookahead_dist;
  double min_approach_linear_velocity;
  double approach_velocity_scaling_dist;
  double max_allowed_time_to_collision_up_to_carrot;
  bool use_regulated_linear_velocity_scaling;
  bool use_cost_regulated_linear_velocity_scaling;
  double cost_scaling_dist;
  double cost_scaling_gain;
  double inflation_cost_scaling_factor;
  double regulated_linear_scaling_min_radius;
  double regulated_linear_scaling_min_speed;
  bool use_fixed_curvature_lookahead;
  double curvature_lookahead_dist;
  bool use_rotate_to_heading;
  double max_angular_accel;
  double max_linear_accel;
  double rotate_to_heading_min_angle;
  bool allow_reversing;
  double max_robot_pose_search_dist;
  bool interpolate_curvature_after_goal;
  bool use_collision_detection;
  double transform_tolerance;
  double goal_dist_tol;
  double angle_tol;
  double max_linear_vel;
  double max_angular_vel;
  double theta_stopped_vel;
  double trans_stopped_vel;
  double k;
  double min_turning_radius;
  bool use_vector_pure_pursuit;
};

/**
 * @class regulated_pure_pursuit_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for RPP
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for regulated_pure_pursuit_controller::ParameterHandler
   */
  ParameterHandler(
    std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node,
    const double costmap_size_x);

  /**
   * @brief Destrructor for regulated_pure_pursuit_controller::ParameterHandler
   */
  ~ParameterHandler();

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  std::shared_ptr<ros::NodeHandle> node_;
  std::shared_ptr<ros::NodeHandle> private_node_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  void dynamicParametersCallback(const RegulatedPurePursuitConfig& cfg, uint32_t level);
  

  std::mutex mutex_;
  std::shared_ptr< dynamic_reconfigure::Server<RegulatedPurePursuitConfig> > dynamic_recfg_server_;
  dynamic_reconfigure::Server<RegulatedPurePursuitConfig>::CallbackType dynamic_recfg_callback_;
  Parameters params_;
};

}  // namespace regulated_pure_pursuit_controller

#endif  // REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_
