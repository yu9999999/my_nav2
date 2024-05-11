#ifndef NAV2_THETA_STAR_PLANNER__THETA_STAR_HPP_
#define NAV2_THETA_STAR_PLANNER__THETA_STAR_HPP_

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
// 小顶堆
struct comp
{
  bool operator()(const tree_node * p1, const tree_node * p2)
  {
    return (p1->f) > (p2->f);
  }
};

namespace theta_star
{
class ThetaStar
{
public:
  coordsM src_{}, dst_{};
  nav2_costmap_2d::Costmap2D * costmap_{};
  /// 成本图遍历成本的权重
  double w_traversal_cost_;
  /// 欧氏距离成本的权重（用于计算 g 成本）
  double w_euc_cost_;
  /// 启发式成本的权重（用于 h 成本计算）
  double w_heuristic_cost_;
  /// 要在4个连接（上、下、左、右）和8个连接（所有相邻单元格）图形扩展之间进行选择，即可接受的值为4和8
  int how_many_corners_;
  /// 是否允许通通往未知区域
  bool allow_unknown_;
  /// 分别是地图的 x 方向和 y 方向的长度
  int size_x_, size_y_;
  ThetaStar();

  ~ThetaStar() = default;
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
  // 统计已经展开的节点
  int nodes_opened = 0;

protected:
  /// 存储坐标点指针索引的容器，存储在node_position_[size_x_ * y + x]位置，感觉node_position_与nodes_data_的区别就是在其索引上
  std::vector<tree_node *> node_position_;
  /// 存储节点数据，存储父节点的指针、成本和索引等，这里的索引是从0开始加的
  std::vector<tree_node> nodes_data_;
  /// 优先队列
  std::priority_queue<tree_node *, std::vector<tree_node *>, comp> queue_;
  /// 已经计算过的节点数目
  int index_generated_;
  // 用于展开节点周围的八个节点，前四个是直向，后四个是斜方向
  const coordsM moves[8] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};

  tree_node * exp_node;
  void resetParent(tree_node * curr_data);
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

  // 代价图的代价值切换范围
  inline double getCost(const int & cx, const int & cy) const
  {
    return 26 + 0.9 * costmap_->getCost(cx, cy);
  }

  // 计算代价图的代价值用于计算g
  inline double getTraversalCost(const int & cx, const int & cy)
  {
    double curr_cost = getCost(cx, cy);
    return w_traversal_cost_ * curr_cost * curr_cost / LETHAL_COST / LETHAL_COST;
  }
  // 欧氏距离用于计算g
  inline double getEuclideanCost(const int & ax, const int & ay, const int & bx, const int & by)
  {
    return w_euc_cost_ * std::hypot(ax - bx, ay - by);
  }
  // 计算启发式H
  inline double getHCost(const int & cx, const int & cy)
  {
    return w_heuristic_cost_ * std::hypot(cx - dst_.x, cy - dst_.y);
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
   /// 插入节点指针索引
  inline void addIndex(const int & cx, const int & cy, tree_node * node_this)
  {
    node_position_[size_x_ * cy + cx] = node_this;
  }
  inline tree_node * getIndex(const int & cx, const int & cy)
  {
    return node_position_[size_x_ * cy + cx];
  }
   /// 插入节点数据
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
}   //  namespace theta_star

#endif  //  NAV2_THETA_STAR_PLANNER__THETA_STAR_HPP_
