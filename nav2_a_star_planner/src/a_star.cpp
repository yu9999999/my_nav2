//  Copyright 2020 Anshumaan Singh
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <vector>
#include "nav2_a_star_planner/a_star.hpp"

namespace a_star
{

AStar::AStar()
: w_traversal_cost_(1.0),
  w_euc_cost_(2.0),
  w_heuristic_cost_(1.0),
  how_many_corners_(8),
  allow_unknown_(true),
  size_x_(0),
  size_y_(0),
  index_generated_(0)
{
  exp_node = new tree_node;
}

void AStar::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  unsigned int s[2], d[2];
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, s[0], s[1]);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, d[0], d[1]);

  src_ = {static_cast<int>(s[0]), static_cast<int>(s[1])};
  dst_ = {static_cast<int>(d[0]), static_cast<int>(d[1])};
}

bool AStar::generatePath(std::vector<coordsW> & raw_path)
{
  // 初始化全局变量的值
  resetContainers();
  // 初始化节点存储容器
  addToNodesData(index_generated_);
  // 计算成本g，启发式H成本,对于起始点来说就是计算遍历成本
  double src_g_cost = getTraversalCost(src_.x, src_.y), src_h_cost = getHCost(src_.x, src_.y);
  // 起始节点
  nodes_data_[index_generated_] =
  {src_.x, src_.y, src_g_cost, src_h_cost, &nodes_data_[index_generated_], true,
    src_g_cost + src_h_cost};
  queue_.push({&nodes_data_[index_generated_]});
  // 存储id
  addIndex(src_.x, src_.y, &nodes_data_[index_generated_]);
  tree_node * curr_data = &nodes_data_[index_generated_];
  index_generated_++;
  nodes_opened = 0;
  // 主循环搜索路径，广度优先搜索
  while (!queue_.empty()) {
    nodes_opened++;
    // 判断是否已经接触到目标点
    if (isGoal(*curr_data)) {
      break;
    }
    // 将curr_data放入至close队列，这里的三段代码虽然没有明确定义close队列，但是经过这三段代码的操作，该节点不会再进入queue_中了
    curr_data->is_in_queue = false;
    queue_.pop();
    setNeighbors(curr_data);
    // 更新curr_data
    curr_data = queue_.top();
  }

  if (queue_.empty()) {
    raw_path.clear();
    return false;
  }
  // 依据父节点回溯获取路径
  backtrace(raw_path, curr_data);
  // 每次执行完毕之后清空优先队列
  clearQueue();

  return true;
}

void AStar::setNeighbors(const tree_node * curr_data)
{
  int mx, my;
  tree_node * m_id = nullptr;
  double g_cost, h_cost, cal_cost;

  for (int i = 0; i < how_many_corners_; i++) {
    mx = curr_data->x + moves[i].x;
    my = curr_data->y + moves[i].y;

    if (withinLimits(mx, my)) {
      if (!isSafe(mx, my)) {
        continue;
      }
    } else {
      continue;
    }

    g_cost = curr_data->g + getEuclideanCost(curr_data->x, curr_data->y, mx, my) +
      getTraversalCost(mx, my);

    m_id = getIndex(mx, my);

    if (m_id == nullptr) {
      addToNodesData(index_generated_);
      m_id = &nodes_data_[index_generated_];
      addIndex(mx, my, m_id);
      index_generated_++;
    }

    exp_node = m_id;

    h_cost = getHCost(mx, my);
    cal_cost = g_cost + h_cost;
    if (exp_node->f > cal_cost) {
      exp_node->g = g_cost;
      exp_node->h = h_cost;
      exp_node->f = cal_cost;
      exp_node->parent_id = curr_data;
      if (!exp_node->is_in_queue) {
        exp_node->x = mx;
        exp_node->y = my;
        exp_node->is_in_queue = true;
        queue_.push({m_id});
      }
    }
  }
}

void AStar::backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const
{
  std::vector<coordsW> path_rev;
  coordsW world{};
  do {
    costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
    path_rev.push_back(world);
    if (path_rev.size() > 1) {
      curr_n = curr_n->parent_id;
    }
  } while (curr_n->parent_id != curr_n);
  costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
  path_rev.push_back(world);

  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

bool AStar::losCheck(
  const int & x0, const int & y0, const int & x1, const int & y1,
  double & sl_cost) const
{
  sl_cost = 0;

  int cx, cy;
  int dy = abs(y1 - y0), dx = abs(x1 - x0), f = 0;
  int sx, sy;
  sx = x1 > x0 ? 1 : -1;
  sy = y1 > y0 ? 1 : -1;

  int u_x = (sx - 1) / 2;
  int u_y = (sy - 1) / 2;
  cx = x0;
  cy = y0;

  if (dx >= dy) {
    while (cx != x1) {
      f += dy;
      if (f >= dx) {
        if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
          return false;
        }
        cy += sy;
        f -= dx;
      }
      if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
        return false;
      }
      if (dy == 0 && !isSafe(cx + u_x, cy, sl_cost) && !isSafe(cx + u_x, cy - 1, sl_cost)) {
        return false;
      }
      cx += sx;
    }
  } else {
    while (cy != y1) {
      f = f + dx;
      if (f >= dy) {
        if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
          return false;
        }
        cx += sx;
        f -= dy;
      }
      if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
        return false;
      }
      if (dx == 0 && !isSafe(cx, cy + u_y, sl_cost) && !isSafe(cx - 1, cy + u_y, sl_cost)) {
        return false;
      }
      cy += sy;
    }
  }
  return true;
}

void AStar::resetContainers()
{
  index_generated_ = 0;
  int last_size_x = size_x_;
  int last_size_y = size_y_;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (((last_size_x != curr_size_x) || (last_size_y != curr_size_y)) &&
    static_cast<int>(node_position_.size()) < (curr_size_x * curr_size_y))
  {
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data_.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
}

void AStar::initializePosn(int size_inc)
{
  if (!node_position_.empty()) {
    for (int i = 0; i < size_x_ * size_y_; i++) {
      node_position_[i] = nullptr;
    }
  }

  for (int i = 0; i < size_inc; i++) {
    node_position_.push_back(nullptr);
  }
}
}  
