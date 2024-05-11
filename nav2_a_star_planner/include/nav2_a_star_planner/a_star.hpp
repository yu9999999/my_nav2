#ifndef NAV2_A_STAR_PLANNER__A_STAR_HPP_
#define NAV2_A_STAR_PLANNER__A_STAR_HPP_

#include <cmath>
#include <chrono>
#include <vector>
#include <queue>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

const double INF_COST = DBL_MAX;
const int UNKNOWN_COST = 255;
const int OBS_COST = 254;
const int LETHAL_COST = 252;

struct coordsM
{
  int x, y;
};

struct coordsW
{
  double x, y;
};

struct tree_node
{
  int x, y;
  double g = INF_COST;
  double h = INF_COST;
  const tree_node * parent_id = nullptr;
  bool is_in_queue = false;
  double f = INF_COST;
};

struct comp
{
  bool operator()(const tree_node * p1, const tree_node * p2)
  {
    return (p1->f) > (p2->f);
  }
};

namespace a_star
{
class AStar
{
public:
  coordsM src_{}, dst_{};
  nav2_costmap_2d::Costmap2D * costmap_{};
  double w_traversal_cost_;
  double w_euc_cost_;
  double w_heuristic_cost_;
  int how_many_corners_;
  bool allow_unknown_;
  int size_x_, size_y_;
  AStar();
  ~AStar() = default;
  bool generatePath(std::vector<coordsW> & raw_path);
  inline bool isSafe(const int & cx, const int & cy) const
  {
    return (costmap_->getCost(
             cx,
             cy) == UNKNOWN_COST && allow_unknown_) || costmap_->getCost(cx, cy) < LETHAL_COST;
  }
  void setStartAndGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
  bool isUnsafeToPlan() const
  {
    return !(isSafe(src_.x, src_.y)) || !(isSafe(dst_.x, dst_.y));
  }

  int nodes_opened = 0;

protected:
  std::vector<tree_node *> node_position_;
  std::vector<tree_node> nodes_data_;
  std::priority_queue<tree_node *, std::vector<tree_node *>, comp> queue_;
  int index_generated_;

  const coordsM moves[8] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};

  tree_node * exp_node;
  void setNeighbors(const tree_node * curr_data);
  bool losCheck(
    const int & x0, const int & y0, const int & x1, const int & y1,
    double & sl_cost) const;
  void backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const;
  bool isSafe(const int & cx, const int & cy, double & cost) const
  {
    double curr_cost = getCost(cx, cy);
    if ((costmap_->getCost(cx, cy) == UNKNOWN_COST && allow_unknown_) || curr_cost < LETHAL_COST) {
      if (costmap_->getCost(cx, cy) == UNKNOWN_COST) {
        curr_cost = OBS_COST - 1;
      }
      cost += w_traversal_cost_ * curr_cost * curr_cost / LETHAL_COST / LETHAL_COST;
      return true;
    } else {
      return false;
    }
  }
  inline double getCost(const int & cx, const int & cy) const
  {
    return 26 + 0.9 * costmap_->getCost(cx, cy);
  }
  inline double getTraversalCost(const int & cx, const int & cy)
  {
    double curr_cost = getCost(cx, cy);
    return w_traversal_cost_ * curr_cost * curr_cost / LETHAL_COST / LETHAL_COST;
  }
  inline double getEuclideanCost(const int & ax, const int & ay, const int & bx, const int & by)
  {
    return w_euc_cost_ * std::hypot(ax - bx, ay - by);
  }
  inline double getHCost(const int & cx, const int & cy)
  {
    return w_heuristic_cost_ * (abs(cx - dst_.x) + abs(cy - dst_.y));
  }
  inline bool withinLimits(const int & cx, const int & cy) const
  {
    return cx >= 0 && cx < size_x_ && cy >= 0 && cy < size_y_;
  }
  inline bool isGoal(const tree_node & this_node) const
  {
    return this_node.x == dst_.x && this_node.y == dst_.y;
  }
  void initializePosn(int size_inc = 0);

  inline void addIndex(const int & cx, const int & cy, tree_node * node_this)
  {
    node_position_[size_x_ * cy + cx] = node_this;
  }

  inline tree_node * getIndex(const int & cx, const int & cy)
  {
    return node_position_[size_x_ * cy + cx];
  }

  void addToNodesData(const int & id_this)
  {
    if (static_cast<int>(nodes_data_.size()) <= id_this) {
      nodes_data_.push_back({});
    } else {
      nodes_data_[id_this] = {};
    }
  }
  void resetContainers();
  void clearQueue()
  {
    queue_ = std::priority_queue<tree_node *, std::vector<tree_node *>, comp>();
  }
};
}   

#endif  
