#include <vector>
#include <memory>
#include <string>
#include "nav2_a_star_planner/a_star_planner.hpp"
#include "nav2_a_star_planner/a_star.hpp"

namespace nav2_a_star_planner
{
void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<a_star::AStar>();
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".how_many_corners", rclcpp::ParameterValue(8));

  node->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

  if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
    planner_->how_many_corners_ = 8;
    RCLCPP_WARN(logger_, "Your value for - .how_many_corners  was overridden, and is now set to 8");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", planner_->allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_euc_cost", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".w_euc_cost", planner_->w_euc_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".w_traversal_cost", planner_->w_traversal_cost_);

  planner_->w_heuristic_cost_ = planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type nav2_a_star_planner", name_.c_str());
  planner_.reset();
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type nav2_a_star_planner", name_.c_str());
  // Add callback for dynamic parameters
  auto node = parent_node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&AStarPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type nav2_a_star_planner", name_.c_str());
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  auto start_time = std::chrono::steady_clock::now();
  unsigned int mx_start, my_start, mx_goal, my_goal;
  planner_->costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);
  planner_->costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);
  if (mx_start == mx_goal && my_start == my_goal) {
    if (planner_->costmap_->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(logger_, "Failed to create a unique pose path because of obstacles");
      return global_path;
    }
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    global_path.poses.push_back(pose);
    return global_path;
  }

  planner_->setStartAndGoal(start, goal);
  RCLCPP_DEBUG(
    logger_, "Got the src and dst... (%i, %i) && (%i, %i)",
    planner_->src_.x, planner_->src_.y, planner_->dst_.x, planner_->dst_.y);
  getPlan(global_path);
  size_t plan_size = global_path.poses.size();
  if (plan_size > 0) {
    global_path.poses.back().pose.orientation = goal.pose.orientation;
  }
  if (use_final_approach_orientation_) {
    if (plan_size == 1) {
      global_path.poses.back().pose.orientation = start.pose.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = global_path.poses.back().pose.position;
      auto approach_pose = global_path.poses[plan_size - 2].pose.position;
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      global_path.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
  }

  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
  return global_path;
}

void AStarPlanner::getPlan(nav_msgs::msg::Path & global_path)
{
  std::vector<coordsW> path;
  if (planner_->isUnsafeToPlan()) {
    RCLCPP_ERROR(logger_, "Either of the start or goal pose are an obstacle! ");
    global_path.poses.clear();
  } else if (planner_->generatePath(path)) {
    geometry_msgs::msg::PoseStamped p1;
    for (unsigned int j = 0; j < path.size() - 1; j++) {
      p1.pose.position.x = path[j].x;
      p1.pose.position.y = path[j].y;
      global_path.poses.push_back(p1);
    }
  } else {
    RCLCPP_ERROR(logger_, "Could not generate path between the given poses");
    global_path.poses.clear();
  }
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;
}

rcl_interfaces::msg::SetParametersResult
AStarPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".how_many_corners") {
        planner_->how_many_corners_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".w_euc_cost") {
        planner_->w_euc_cost_ = parameter.as_double();
      } else if (name == name_ + ".w_traversal_cost") {
        planner_->w_traversal_cost_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        planner_->allow_unknown_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_a_star_planner::AStarPlanner, nav2_core::GlobalPlanner)
