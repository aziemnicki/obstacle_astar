// Copyright 2024 Andrzej_Norbert_Jeremiasz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OBSTACLE_ASTAR__OBSTACLE_ASTAR_NODE_HPP_
#define OBSTACLE_ASTAR__OBSTACLE_ASTAR_NODE_HPP_

#include "freespace_planning_algorithms/abstract_algorithm.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"

#include <freespace_planning_algorithms/astar_search.hpp>
#include <rclcpp/rclcpp.hpp>

#include <motion_utils/trajectory/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>


namespace obstacle_astar
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using freespace_planning_algorithms::AbstractPlanningAlgorithm;
using freespace_planning_algorithms::AstarParam;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::PlannerCommonParam;
using freespace_planning_algorithms::VehicleShape;
using freespace_planning_algorithms::PlannerWaypoint;
using freespace_planning_algorithms::PlannerWaypoints;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double vehicle_shape_margin_m;
  int collision_check_distance;
  int planning_distance;
};

class ObstacleAstarNode : public rclcpp::Node
{
public:
  explicit ObstacleAstarNode(const rclcpp::NodeOptions & node_options);

private:
  // ros 
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;

  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // params
  NodeParam node_param_;
  VehicleShape vehicle_shape_;

  // variables
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;
  PoseStamped goal_pose_;
  Trajectory current_trajectory_;
  bool is_trajectory_saved_ = false; 
  bool is_avoiding_ = false; 
  
  Trajectory::ConstSharedPtr input_trajectory_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Odometry::ConstSharedPtr odom_;

  OccupancyGrid::ConstSharedPtr partial_grid_;

  // functions used in the constructor
  PlannerCommonParam getPlannerCommonParam();

  // functions, callback
  void onTrajectory(const Trajectory::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);

  void onTimer();

  bool isPlanRequired();
  void PlanTrajectory();
  void initializePlanningAlgorithm();

  void extractPartialGridMap();

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;

};
}  // namespace obstacle_astar

#endif  // OBSTACLE_ASTAR__OBSTACLE_ASTAR_NODE_HPP_
