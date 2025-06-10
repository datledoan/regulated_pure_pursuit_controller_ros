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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "regulated_pure_pursuit_controller/parameter_handler.hpp"

namespace regulated_pure_pursuit_controller
{

ParameterHandler::ParameterHandler(
  std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node,
  const double costmap_size_x)
{
  node_ = node;
  private_node_ = private_node;

  private_node_->param<double>("desired_linear_vel", params_.desired_linear_vel, 0.5);
  private_node_->param<double>("lookahead_dist", params_.lookahead_dist, 0.8);
  private_node_->param<double>("rotate_to_heading_angular_vel", params_.rotate_to_heading_angular_vel, 0.5);
  private_node_->param<double>("max_lookahead_dist", params_.max_lookahead_dist, 1.0);
  private_node_->param<double>("min_lookahead_dist", params_.min_lookahead_dist, 0.5);
  private_node_->param<double>("lookahead_time", params_.lookahead_time, 4.0);
  private_node_->param<bool>("use_velocity_scaled_lookahead_dist", params_.use_velocity_scaled_lookahead_dist, true);
  private_node_->param<double>("min_approach_linear_velocity", params_.min_approach_linear_velocity, 0.1);
  private_node_->param<double>("approach_velocity_scaling_dist", params_.approach_velocity_scaling_dist, 0.5);
  
  if (params_.approach_velocity_scaling_dist > costmap_size_x / 2.0) {
    ROS_WARN("[RPP] approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }

  private_node_->param<double>("max_allowed_time_to_collision_up_to_carrot", params_.max_allowed_time_to_collision_up_to_carrot, 2.0);
  private_node_->param<bool>("use_regulated_linear_velocity_scaling", params_.use_regulated_linear_velocity_scaling, false);
  private_node_->param<bool>("use_cost_regulated_linear_velocity_scaling", params_.use_cost_regulated_linear_velocity_scaling, false);
  private_node_->param<double>("cost_scaling_dist", params_.cost_scaling_dist, 0.5);
  private_node_->param<double>("cost_scaling_gain", params_.cost_scaling_gain, 1.0);
  private_node_->param<double>("inflation_cost_scaling_factor", params_.inflation_cost_scaling_factor, 3.0);
  private_node_->param<double>("regulated_linear_scaling_min_radius", params_.regulated_linear_scaling_min_radius, 0.5);
  private_node_->param<double>("regulated_linear_scaling_min_speed", params_.regulated_linear_scaling_min_speed, 0.1);
  private_node_->param<bool>("use_fixed_curvature_lookahead", params_.use_fixed_curvature_lookahead, false);
  private_node_->param<double>("curvature_lookahead_dist", params_.curvature_lookahead_dist, 0.5);
  private_node_->param<bool>("use_rotate_to_heading", params_.use_rotate_to_heading, false);
  private_node_->param<double>("max_angular_accel", params_.max_angular_accel, 0.5);
  private_node_->param<double>("max_linear_accel", params_.max_linear_accel, 0.5);
  private_node_->param<double>("rotate_to_heading_min_angle", params_.rotate_to_heading_min_angle, 0.1);
  private_node_->param<bool>("allow_reversing", params_.allow_reversing, false);
  private_node_->param<double>("max_robot_pose_search_dist", params_.max_robot_pose_search_dist, 1.0);
  private_node_->param<bool>("interpolate_curvature_after_goal", params_.interpolate_curvature_after_goal, false);
  private_node_->param<bool>("use_collision_detection", params_.use_collision_detection, false);
  private_node_->param<double>("transform_tolerance", params_.transform_tolerance, 0.1);
  private_node_->param<double>("goal_dist_tol", params_.goal_dist_tol, 0.2);
  private_node_->param<double>("angle_tol", params_.angle_tol, 0.1);
  private_node_->param<double>("max_linear_vel", params_.max_linear_vel, 0.5);
  private_node_->param<double>("max_angular_vel", params_.max_angular_vel, 0.5);
  private_node_->param<double>("theta_stopped_vel", params_.theta_stopped_vel, 0.1);
  private_node_->param<double>("trans_stopped_vel",params_.trans_stopped_vel, 0.1);
  private_node_->param<double>("k", params_.k, 5.0);
  private_node_->param<double>("min_turning_radius", params_.min_turning_radius, 0.25);
  private_node_->param<bool>("use_vector_pure_pursuit", params_.use_vector_pure_pursuit, false);

  // Dynamic reconfigure
  dynamic_recfg_server_ = std::make_shared< dynamic_reconfigure::Server<RegulatedPurePursuitConfig> >(*private_node_);
  dynamic_recfg_callback_ = boost::bind(&ParameterHandler::dynamicParametersCallback, this, _1, _2);
  dynamic_recfg_server_->setCallback(dynamic_recfg_callback_);
}

void ParameterHandler::dynamicParametersCallback(const RegulatedPurePursuitConfig& cfg, uint32_t level)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  params_.desired_linear_vel = cfg.desired_linear_vel;
  params_.lookahead_dist = cfg.lookahead_dist;
  params_.rotate_to_heading_angular_vel = cfg.rotate_to_heading_angular_vel;
  params_.max_lookahead_dist = cfg.max_lookahead_dist;
  params_.min_lookahead_dist = cfg.min_lookahead_dist;
  params_.lookahead_time = cfg.lookahead_time;
  params_.use_velocity_scaled_lookahead_dist = cfg.use_velocity_scaled_lookahead_dist;
  params_.min_approach_linear_velocity = cfg.min_approach_linear_velocity;
  params_.approach_velocity_scaling_dist = cfg.approach_velocity_scaling_dist;
  params_.max_allowed_time_to_collision_up_to_carrot = cfg.max_allowed_time_to_collision_up_to_carrot;
  params_.use_regulated_linear_velocity_scaling = cfg.use_regulated_linear_velocity_scaling;
  params_.use_cost_regulated_linear_velocity_scaling = cfg.use_cost_regulated_linear_velocity_scaling;
  params_.cost_scaling_dist = cfg.cost_scaling_dist;
  params_.cost_scaling_gain = cfg.cost_scaling_gain;
  params_.inflation_cost_scaling_factor = cfg.inflation_cost_scaling_factor;
  params_.regulated_linear_scaling_min_radius = cfg.regulated_linear_scaling_min_radius;
  params_.regulated_linear_scaling_min_speed = cfg.regulated_linear_scaling_min_speed;
  params_.use_fixed_curvature_lookahead = cfg.use_fixed_curvature_lookahead;
  params_.curvature_lookahead_dist = cfg.curvature_lookahead_dist;
  params_.use_rotate_to_heading = cfg.use_rotate_to_heading;
  params_.max_angular_accel = cfg.max_angular_accel;
  params_.max_linear_accel = cfg.max_linear_accel;
  params_.rotate_to_heading_min_angle = cfg.rotate_to_heading_min_angle;
  params_.allow_reversing = cfg.allow_reversing;
  params_.max_robot_pose_search_dist = cfg.max_robot_pose_search_dist;
  params_.interpolate_curvature_after_goal = cfg.interpolate_curvature_after_goal;
  params_.use_collision_detection = cfg.use_collision_detection;
  params_.transform_tolerance = cfg.transform_tolerance;
  params_.goal_dist_tol = cfg.goal_dist_tol;
  params_.angle_tol = cfg.angle_tol;
  params_.max_linear_vel = cfg.max_linear_vel;
  params_.max_angular_vel = cfg.max_angular_vel;
  params_.theta_stopped_vel = cfg.theta_stopped_vel;
  params_.trans_stopped_vel = cfg.trans_stopped_vel;
  params_.k = cfg.k;
  params_.min_turning_radius = cfg.min_turning_radius;
  params_.use_vector_pure_pursuit = cfg.use_vector_pure_pursuit;
}

ParameterHandler::~ParameterHandler()
{

}

}  // namespace regulated_pure_pursuit_controller
