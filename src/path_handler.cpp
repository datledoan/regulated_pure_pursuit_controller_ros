#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <tf2/utils.h>

#include "regulated_pure_pursuit_controller/path_handler.hpp"
#include "regulated_pure_pursuit_controller/geometry_utils.h"

namespace regulated_pure_pursuit_controller
{

using regulated_pure_pursuit_controller::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
  ros::Duration transform_tolerance,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros)
: transform_tolerance_(transform_tolerance), tf_(tf), costmap_ros_(costmap_ros)
{
}

double PathHandler::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

nav_msgs::Path PathHandler::transformGlobalPlan(
  const geometry_msgs::PoseStamped & pose,
  double max_robot_pose_search_dist,
  bool reject_unit_path)
{
  if (global_plan_.poses.empty()) {
    throw std::runtime_error("[RPP] Received plan with zero length");
  }

  if (reject_unit_path && global_plan_.poses.size() == 1) {
    throw std::runtime_error("[RPP] Received plan with length of one");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw std::runtime_error("[RPP] Unable to transform robot pose into global plan's frame");
  }

  auto closest_pose_upper_bound =
    regulated_pure_pursuit_controller::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    regulated_pure_pursuit_controller::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Make sure we always have at least 2 points on the transformed plan and that we don't prune
  // the global plan below 2 points in order to have always enough point to interpolate the
  // end of path direction
  if (global_plan_.poses.begin() != closest_pose_upper_bound && global_plan_.poses.size() > 1 &&
    transformation_begin == std::prev(closest_pose_upper_bound))
  {
    transformation_begin = std::prev(std::prev(closest_pose_upper_bound));
  }

  // We'll discard points on the plan that are outside the local costmap
  const double max_costmap_extent = getCostmapMaxExtent();
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
        throw std::runtime_error("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("[RPP] Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string frame,
  const geometry_msgs::PoseStamped & in_pose,
  geometry_msgs::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    ROS_ERROR("[RPP] Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace regulated_pure_pursuit_controller
