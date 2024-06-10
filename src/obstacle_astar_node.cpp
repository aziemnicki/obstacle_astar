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

#include "obstacle_astar/obstacle_astar_node.hpp"

namespace obstacle_astar
{
PoseArray trajectory2PoseArray(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

Trajectory getPartialTrajectory(
  const Trajectory & trajectory, const size_t start_index, const size_t length)
{
  Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = rclcpp::Clock().now();

  partial_trajectory.points.reserve(length);
  for (size_t i = 0; i < length; ++i) {
    const size_t index = (start_index + i) % trajectory.points.size();
    partial_trajectory.points.push_back(trajectory.points.at(index));
  }

  return partial_trajectory;
}

Trajectory createTrajectory(
  const PoseStamped & start_pose, const PlannerWaypoints & planner_waypoints,
  const double & velocity)
{
  Trajectory trajectory;
  trajectory.header = planner_waypoints.header;

  for (const auto & awp : planner_waypoints.waypoints) {
    TrajectoryPoint point;

    point.pose = awp.pose.pose;

    point.pose.position.z = start_pose.pose.position.z;  // height = const
    point.longitudinal_velocity_mps = velocity / 3.6;      // velocity = const

    trajectory.points.push_back(point);
  }

  return trajectory;
}

Trajectory replaceTrajectorySegment(
  const Trajectory & current_trajectory, const Trajectory & avoidance_trajectory, 
  const size_t start_index, const size_t goal_index, const size_t planning_distance)
{
  Trajectory modified_trajectory = current_trajectory;

  if (start_index < goal_index) {
    modified_trajectory.points.erase(modified_trajectory.points.begin() + start_index, modified_trajectory.points.begin() + goal_index);
    modified_trajectory.points.insert(modified_trajectory.points.begin() + start_index, avoidance_trajectory.points.begin(), avoidance_trajectory.points.end());
  } else {
    const size_t points_to_remove = planning_distance - (modified_trajectory.points.size() - start_index);
    modified_trajectory.points.erase(modified_trajectory.points.begin() + start_index, modified_trajectory.points.end());
    modified_trajectory.points.erase(modified_trajectory.points.begin(), modified_trajectory.points.begin() + points_to_remove);
    modified_trajectory.points.insert(modified_trajectory.points.begin(), avoidance_trajectory.points.begin(), avoidance_trajectory.points.end());
  }

  return modified_trajectory;
}

bool isNearGoal(const Pose & current_pose, const Pose & goal_pose, const double th_arrived_distance_m)
{
  const double distance_to_goal = tier4_autoware_utils::calcDistance2d(current_pose, goal_pose);
  return distance_to_goal < th_arrived_distance_m;
}

ObstacleAstarNode::ObstacleAstarNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_astar", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
    p.update_rate = declare_parameter<double>("update_rate");
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
    p.collision_check_distance = declare_parameter<int>("collision_check_distance");
    p.planning_distance = declare_parameter<int>("planning_distance");
  }

  // set vehicle_info
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
  }

  // Planning
  initializePlanningAlgorithm();

  // Subscribers
  {
    trajectory_sub_ = create_subscription<Trajectory>(
      "~/input/trajectory", rclcpp::QoS{1}, std::bind(&ObstacleAstarNode::onTrajectory, this, _1));
    occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
      "~/input/occupancy_grid", 1, std::bind(&ObstacleAstarNode::onOccupancyGrid, this, _1));
    odom_sub_ = create_subscription<Odometry>(
      "~/input/odometry", 100, std::bind(&ObstacleAstarNode::onOdometry, this, _1));
  }

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", rclcpp::QoS(1000));
  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&ObstacleAstarNode::onTimer, this));
  }

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

PlannerCommonParam ObstacleAstarNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");
  p.minimum_turning_radius = declare_parameter<double>("minimum_turning_radius");
  p.maximum_turning_radius = declare_parameter<double>("maximum_turning_radius");
  p.turning_radius_size = declare_parameter<int>("turning_radius_size");
  p.maximum_turning_radius = std::max(p.maximum_turning_radius, p.minimum_turning_radius);
  p.turning_radius_size = std::max(p.turning_radius_size, 1);

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

void ObstacleAstarNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  if (!is_trajectory_saved_) {
    input_trajectory_ = msg;
    current_trajectory_ = *msg;
    is_trajectory_saved_ = true;
  }
}

void ObstacleAstarNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

void ObstacleAstarNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

bool ObstacleAstarNode::isPlanRequired()
{
  const size_t start_index = motion_utils::findNearestIndex(current_trajectory_.points, current_pose_.pose.position);
  const auto forward_trajectory = getPartialTrajectory(current_trajectory_, start_index, node_param_.collision_check_distance);

  algo_->setMap(*occupancy_grid_);
  const bool is_obstacle_found = algo_->hasObstacleOnTrajectory(trajectory2PoseArray(forward_trajectory));

  return is_obstacle_found;
}

void ObstacleAstarNode::PlanTrajectory()
{
  const size_t start_index = motion_utils::findNearestIndex(current_trajectory_.points, current_pose_.pose.position);
  PoseStamped start_pose;
  start_pose.pose = current_trajectory_.points.at(start_index).pose;

  const size_t goal_index = (start_index + node_param_.planning_distance) % current_trajectory_.points.size();
  goal_pose_.pose = current_trajectory_.points.at(goal_index).pose;

  extractPartialGridMap();
  algo_->setMap(*partial_grid_);

  const rclcpp::Time start = get_clock()->now();
  const bool result = algo_->makePlan(start_pose.pose, goal_pose_.pose);
  const rclcpp::Time end = get_clock()->now();

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array = algo_->getMarkerArray();
  markers_pub_->publish(marker_array);

  if (result) {
    is_avoiding_ = true;

    const auto avoidance_trajectory = createTrajectory(start_pose, algo_->getWaypoints(), node_param_.waypoints_velocity);
    const auto modified_trajectory = replaceTrajectorySegment(current_trajectory_, avoidance_trajectory, start_index, goal_index, node_param_.planning_distance);
    current_trajectory_ = modified_trajectory;

  } 
}

void ObstacleAstarNode::onTimer()
{
  // Check all inputs are ready
  if (!occupancy_grid_ || !input_trajectory_ || !odom_) {
    return;
  }

  // Get current pose
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;
  
  if (isPlanRequired()) {
    PlanTrajectory();
  } else {
    if (is_avoiding_ && isNearGoal(current_pose_.pose, goal_pose_.pose, node_param_.th_arrived_distance_m)) {
      is_avoiding_ = false;
      current_trajectory_ = *input_trajectory_;
    }
  }
  trajectory_pub_->publish(current_trajectory_);
}

void ObstacleAstarNode::initializePlanningAlgorithm()
{
  // Extend robot shape
  freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;

  const auto planner_common_param = getPlannerCommonParam();

  const auto algo_name = node_param_.planning_algorithm;

  // initialize specified algorithm
  if (algo_name == "astar") {
    algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);
}

void ObstacleAstarNode::extractPartialGridMap()
{
  PoseStamped internal_goal = PoseStamped();

  const size_t start_index_forward = motion_utils::findNearestIndex(current_trajectory_.points, current_pose_.pose.position);
  const size_t goal_index = (start_index_forward + 30) % current_trajectory_.points.size();
  internal_goal.pose = current_trajectory_.points[goal_index].pose;
      
  
  // Rectangle dimensions
  double width = 220;
  double height = 130;

  // Calculate the center of the rectangle
  double centerX = (current_pose_.pose.position.x + internal_goal.pose.position.x) / 2;
  double centerY = (current_pose_.pose.position.y + internal_goal.pose.position.y) / 2;

  // Occupancy grid parameters
  double resolution = occupancy_grid_->info.resolution;
  double originX = occupancy_grid_->info.origin.position.x;
  double originY = occupancy_grid_->info.origin.position.y;
  int gridWidth = occupancy_grid_->info.width;
  int gridHeight = occupancy_grid_->info.height;

  // Create a new occupancy grid for the partial map
  nav_msgs::msg::OccupancyGrid partial_grid_map;
  partial_grid_map.header = occupancy_grid_->header;
  partial_grid_map.info.resolution = resolution;
  partial_grid_map.info.width = width;
  partial_grid_map.info.height = height;
  partial_grid_map.info.origin.position.x = centerX - (width * resolution) / 2;
  partial_grid_map.info.origin.position.y = centerY - (height * resolution) / 2;
  partial_grid_map.data.resize(width * height, -1);

  // Extract the partial grid map from the original occupancy grid
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      // Partial grid point in global coordinates
      double globalX = partial_grid_map.info.origin.position.x + i * resolution;
      double globalY = partial_grid_map.info.origin.position.y + j * resolution;

      // Convert to original grid indices
      int gridX = (globalX - originX) / resolution;
      int gridY = (globalY - originY) / resolution;

      // Check if indices are within bounds
      if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        int original_index = gridY * gridWidth + gridX;
        int partial_index = j * width + i;
        partial_grid_map.data[partial_index] = occupancy_grid_->data[original_index];
      }
    }
  }

  // Publish the partial grid map
  partial_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(partial_grid_map);
}

}  // namespace obstacle_astar

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_astar::ObstacleAstarNode)