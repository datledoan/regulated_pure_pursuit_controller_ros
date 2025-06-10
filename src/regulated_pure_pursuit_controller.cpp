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
#include <tf2/utils.h>

#include "angles/angles.h"
#include "regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "regulated_pure_pursuit_controller/geometry_utils.h"
#include <pluginlib/class_list_macros.h>

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace costmap_2d;  // NOLINT

PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController, mbf_costmap_core::CostmapController)

namespace regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::initialize(
  std::string name, tf2_ros::Buffer* tf,
  costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!isInitialized() ){
    node_ = std::make_shared<ros::NodeHandle>("");
    private_node_ = std::make_shared<ros::NodeHandle>("~" + name);
    costmap_ros_ = std::shared_ptr<costmap_2d::Costmap2DROS>(costmap_ros);
    costmap_ = costmap_ros_->getCostmap();
    tf_ = std::shared_ptr<tf2_ros::Buffer>(tf);
    odom_helper_.setOdomTopic( odom_topic_ );

    // Handles storage and dynamic configuration of parameters.
    // Returns pointer to data current param settings.
    param_handler_ = std::make_unique<ParameterHandler>(
      node_, private_node_, costmap_->getSizeInMetersX());
    params_ = param_handler_->getParams();

    // Handles global path transformations
    path_handler_ = std::make_unique<PathHandler>(
      ros::Duration(params_->transform_tolerance), tf_, costmap_ros_);

    // Checks for imminent collisions
    collision_checker_ = std::make_unique<CollisionChecker>(node_, costmap_ros_, params_);

    double control_frequency = 20.0;
    goal_dist_tol_ = 0.25;  // reasonable default before first update

    node_->getParam("controller_frequency", control_frequency);
    control_duration_ = 1.0 / control_frequency;

    global_path_pub_ = private_node_->advertise<nav_msgs::Path>("received_global_plan", 1);
    global_path_origin_pub_ = private_node_->advertise<nav_msgs::Path>("global_plan_origin", 1);
    carrot_pub_ = private_node_->advertise<geometry_msgs::PointStamped>("lookahead_point", 1);
    vector_pub_ = private_node_->advertise<geometry_msgs::PoseStamped>("lookahead_vector", 1);
    curvature_carrot_pub_ = private_node_->advertise<geometry_msgs::PointStamped>(
      "curvature_lookahead_point", 1);
    
    initialized_ = true;
    ROS_INFO("[RPP] RegulatedPurePursuitController initialized.");
  }

  else{
    ROS_WARN("[RPP] RegulatedPurePursuitController already initialized.");
  }

}

geometry_msgs::PointStamped RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::PoseStamped & carrot_pose)
{
  auto carrot_msg = geometry_msgs::PointStamped();
  carrot_msg.header = carrot_pose.header;
  carrot_msg.point.x = carrot_pose.pose.position.x;
  carrot_msg.point.y = carrot_pose.pose.position.y;
  carrot_msg.point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = params_->lookahead_dist;
  if (params_->use_velocity_scaled_lookahead_dist) {
    lookahead_dist = fabs(speed.linear.x) * params_->lookahead_time;
    lookahead_dist = std::clamp(
      lookahead_dist, params_->min_lookahead_dist, params_->max_lookahead_dist);
  }

  return lookahead_dist;
}

double calculateCurvature(geometry_msgs::Point lookahead_point)
{
  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (lookahead_point.x * lookahead_point.x) +
    (lookahead_point.y * lookahead_point.y);

  // Find curvature of circle (k = 1 / R)
  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_point.y / carrot_dist2;
  } else {
    return 0.0;
  }
}

double RegulatedPurePursuitController::calcTurningRadius(const geometry_msgs::PoseStamped & target_pose)
{
  // Calculate angle to lookahead point
  double target_angle = angles::normalize_angle(tf2::getYaw(target_pose.pose.orientation));
  double distance = std::hypot(target_pose.pose.position.x, target_pose.pose.position.y);
  // Compute turning radius (screw center)
  double turning_radius;
  if (params_->allow_reversing || target_pose.pose.position.x >= 0.0) {
    if (std::abs(target_pose.pose.position.y) > 1e-6) {
      double phi_1 = std::atan2(
        (2 * std::pow(target_pose.pose.position.y, 2) - std::pow(distance, 2)),
        (2 * target_pose.pose.position.x * target_pose.pose.position.y));
      double term_2 = std::pow(distance, 2) / (2 * target_pose.pose.position.y);
      double phi_2 = std::atan2(term_2, 0.0);
      double phi = angles::normalize_angle_positive(phi_1 - phi_2);
      phi = std::max(phi, 1.0e-9);
      double term_1 = (params_->k * phi) / (((params_->k - 1) * phi) + target_angle);
      turning_radius = std::abs(term_1 * term_2);
    } else {
      // Handle case when target is directly ahead
      turning_radius = std::numeric_limits<double>::max();
    }
  } else {
    // If lookahead point is behind the robot, set turning radius to minimum
    turning_radius = params_->min_turning_radius;
  }

  // Limit turning radius to avoid extremely sharp turns
  turning_radius = std::max(turning_radius, params_->min_turning_radius);
  if (target_pose.pose.position.y < 0) {
    turning_radius *= -1;
  }
  return turning_radius;
}

geometry_msgs::Quaternion RegulatedPurePursuitController::getOrientation(
  const geometry_msgs::Point & p1,
  const geometry_msgs::Point & p2)
{
  tf2::Quaternion tf2_quat;

  double yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
  tf2_quat.setRPY(0.0, 0.0, yaw);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(tf2_quat);

  return quat_msg;
}

void RegulatedPurePursuitController::getRobotVel(geometry_msgs::Twist& speed){
        nav_msgs::Odometry robot_odom;

        odom_helper_.getOdom(robot_odom);

        speed.linear.x = robot_odom.twist.twist.linear.x;
        speed.angular.z = robot_odom.twist.twist.angular.z;
    }

bool RegulatedPurePursuitController::isThetaGoalReached(double dtheta, double angle_tolerance, 
                                                        double max_angular_vel, double dt)
{
    if (fabs(dtheta) < angle_tolerance || fabs(dtheta) < max_angular_vel * dt)
    {
        return true;
    }
    return false;
}

bool RegulatedPurePursuitController::isGoalReached()
{
  if (goal_reached_){
      ROS_INFO("[RPP] Goal Reached!");
      return true;
  }
  return false;
}

bool RegulatedPurePursuitController::isGoalReached(double xy_tolerance, double yaw_tolerance)
{
  if (goal_reached_){
      ROS_INFO("[RPP] Goal Reached!");
      return true;
  }
  return false;
}

uint32_t RegulatedPurePursuitController::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                  const geometry_msgs::TwistStamped& velocity,
                                  geometry_msgs::TwistStamped& cmd_vel,
                                  std::string& message) 
{
  if(!initialized_)
  {
      ROS_ERROR("[RPP] RegulatedPurePursuitController has not been initialized");
      message = "RegulatedPurePursuitController has not been initialized";
      return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  goal_dist_tol_ = params_->goal_dist_tol;
  angle_tolerance_ = params_->angle_tol;
  max_angular_vel_ = params_->max_angular_vel;
  theta_stopped_vel_ = params_->theta_stopped_vel;
  trans_stopped_vel_ = params_->trans_stopped_vel;
  goal_reached_ = false;

  // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist, params_->interpolate_curvature_after_goal);
  global_path_pub_.publish(transformed_plan);

  // Get current robot velocity
  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);
  geometry_msgs::Twist speed = base_odom.twist.twist;

  geometry_msgs::PoseStamped global_goal = transformed_plan.poses.back();
  double dx_2 = global_goal.pose.position.x * global_goal.pose.position.x;
  double dy_2 = global_goal.pose.position.y * global_goal.pose.position.y;
  double dtheta = angles::normalize_angle(tf2::getYaw(global_goal.pose.orientation));

  if(fabs(std::sqrt(dx_2 + dy_2)) < goal_dist_tol_ && isThetaGoalReached(dtheta, angle_tolerance_, max_angular_vel_, control_duration_) && base_local_planner::stopped(base_odom, theta_stopped_vel_, trans_stopped_vel_))
  {
      goal_reached_ = true;
      return mbf_msgs::ExePathResult::SUCCESS;
  }

  if (transformed_plan.poses.empty())
  {
      ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
      message = "Transformed plan is empty";
      return mbf_msgs::ExePathResult::INVALID_PATH;
  }

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (params_->allow_reversing) {
    // Cusp check
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  auto rotate_to_path_carrot_pose = carrot_pose;
  if(params_->use_vector_pure_pursuit){
    vector_pub_.publish(carrot_pose);
  }
  else{
    carrot_pub_.publish(createCarrotMsg(carrot_pose));
  }

  double linear_vel, angular_vel;
  double lookahead_curvature;
  if(params_->use_vector_pure_pursuit){
    // Implement vector pure pursuit
    double turning_radius = calcTurningRadius(carrot_pose);
    lookahead_curvature = 1.0 / turning_radius;
  }
  else{
    lookahead_curvature = calculateCurvature(carrot_pose.pose.position);
  }
  
  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = getLookAheadPoint(
      params_->curvature_lookahead_dist,
      transformed_plan, params_->interpolate_curvature_after_goal);
    rotate_to_path_carrot_pose = curvature_lookahead_pose;
    
    if(params_->use_vector_pure_pursuit){
      // Implement vector pure pursuit
      double turning_radius = calcTurningRadius(curvature_lookahead_pose);
      regulation_curvature = 1.0 / turning_radius;
    }
    else{
      regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
    }

    curvature_carrot_pub_.publish(createCarrotMsg(curvature_lookahead_pose));
  }

  // Setting the velocity direction
  double x_vel_sign = 1.0;
  if (params_->allow_reversing) {
    x_vel_sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = params_->desired_linear_vel;

  // Make sure we're in compliance with basic constraints
  // For shouldRotateToPath, using x_vel_sign in order to support allow_reversing
  // and rotate_to_path_carrot_pose for the direction carrot pose:
  //        - equal to "normal" carrot_pose when curvature_lookahead_pose = false
  //        - otherwise equal to curvature_lookahead_pose (which can be interpolated after goal)
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    is_rotating_to_heading_ = true;
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(rotate_to_path_carrot_pose, angle_to_heading, x_vel_sign)) {
    is_rotating_to_heading_ = true;
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    is_rotating_to_heading_ = false;
    applyConstraints(
      regulation_curvature, speed,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, x_vel_sign);
      
    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * regulation_curvature;
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (params_->use_collision_detection &&
    collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist))
  {
    ROS_ERROR("[RPP] RegulatedPurePursuitController detected collision ahead!");
  }

  linear_vel = std::clamp(linear_vel, -params_->max_linear_vel, params_->max_linear_vel);
  angular_vel = std::clamp(angular_vel, -params_->max_angular_vel, params_->max_angular_vel);

  // populate and return message
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool RegulatedPurePursuitController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    std::string dummy_message;
    geometry_msgs::PoseStamped dummy_pose;
    geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
    costmap_ros_->getRobotPose(dummy_pose);
    uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
    cmd_vel = cmd_vel_stamped.twist;
    return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path,
  double & x_vel_sign)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  // In case we are reversing
  if (x_vel_sign < 0.0) {
    angle_to_path = angles::normalize_angle(angle_to_path + M_PI);
  }
  return params_->use_rotate_to_heading &&
         fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return params_->use_rotate_to_heading && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * params_->rotate_to_heading_angular_vel;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - params_->max_angular_accel * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + params_->max_angular_accel * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

geometry_msgs::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::Point & p1,
  const geometry_msgs::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::Path & transformed_plan,
  bool interpolate_after_goal)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });
  
  geometry_msgs::PoseStamped pose;

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    if (interpolate_after_goal) {
      auto last_pose_it = std::prev(transformed_plan.poses.end());
      auto prev_last_pose_it = std::prev(last_pose_it);

      double end_path_orientation = atan2(
        last_pose_it->pose.position.y - prev_last_pose_it->pose.position.y,
        last_pose_it->pose.position.x - prev_last_pose_it->pose.position.x);

      // Project the last segment out to guarantee it is beyond the look ahead
      // distance
      auto projected_position = last_pose_it->pose.position;
      projected_position.x += cos(end_path_orientation) * lookahead_dist;
      projected_position.y += sin(end_path_orientation) * lookahead_dist;

      // Use the circle intersection to find the position at the correct look
      // ahead distance
      const auto interpolated_position = circleSegmentIntersection(
        last_pose_it->pose.position, projected_position, lookahead_dist);

      pose.header = last_pose_it->header;
      pose.pose.position = interpolated_position;
      pose.pose.orientation = getOrientation(
        last_pose_it->pose.position, interpolated_position);
    } else {
      goal_pose_it = std::prev(transformed_plan.poses.end());
      pose = *(goal_pose_it);
      pose.pose.orientation = getOrientation(
        std::prev(goal_pose_it)->pose.position, goal_pose_it->pose.position);
    }
  } else if (goal_pose_it == transformed_plan.poses.begin()) {
    pose = *(goal_pose_it); 
    pose.pose.orientation = getOrientation(
      goal_pose_it->pose.position, std::next(goal_pose_it)->pose.position);
  } else{
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    pose.pose.orientation = getOrientation(prev_pose_it->pose.position, point);
  }

  return pose;
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::Twist & curr_speed,
  const double & pose_cost, const nav_msgs::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel, cost_vel = linear_vel, smooth_vel = linear_vel;

  // limit the linear velocity by curvature
  if (params_->use_regulated_linear_velocity_scaling) {
    curvature_vel = heuristics::curvatureConstraint(
      linear_vel, curvature, params_->regulated_linear_scaling_min_radius);
  }

  // limit the linear velocity by proximity to obstacles
  if (params_->use_cost_regulated_linear_velocity_scaling) {
    cost_vel = heuristics::costConstraint(linear_vel, pose_cost, costmap_ros_, params_);
  }

  smooth_vel = std::abs(curr_speed.linear.x) + params_->max_linear_accel * control_duration_;

  // Use the lowest of the 2 constraints, but above the minimum translational speed
  linear_vel = std::min({cost_vel, curvature_vel, smooth_vel});
  linear_vel = std::max(linear_vel, params_->regulated_linear_scaling_min_speed);

  // Apply constraint to reduce speed on approach to the final goal pose
  linear_vel = heuristics::approachVelocityConstraint(
    linear_vel, path, params_->min_approach_linear_velocity,
    params_->approach_velocity_scaling_dist);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->desired_linear_vel);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path)
{
    path.header = plan[0].header;
    for (int i = 0; i < plan.size(); i++){
        path.poses.push_back(plan[i]);
    }
}

bool RegulatedPurePursuitController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if(!initialized_)
  {
      ROS_ERROR("[RPP] RegulatedPurePursuitController has not been initialized, please call initialize() before using this planner");
      return false;
  }

  global_plan_.clear();
  goal_reached_ = false;
  global_plan_ = plan;
  nav_msgs::Path path;
  createPathMsg(plan, path);
  global_path_origin_pub_.publish(path);
  path_handler_->setPlan(path);
  return true;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existence of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    const double dot_prod = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_prod < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }

    if (
      (hypot(oa_x, oa_y) == 0.0 &&
      transformed_plan.poses[pose_id - 1].pose.orientation !=
      transformed_plan.poses[pose_id].pose.orientation)
      ||
      (hypot(ab_x, ab_y) == 0.0 &&
      transformed_plan.poses[pose_id].pose.orientation !=
      transformed_plan.poses[pose_id + 1].pose.orientation))
    {
      // returning the distance since the points overlap
      // but are not simply duplicate points (e.g. in place rotation)
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}
}  // namespace regulated_pure_pursuit_controller
