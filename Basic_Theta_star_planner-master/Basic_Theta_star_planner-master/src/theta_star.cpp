#include <vector>
#include "nav2_theta_star_planner/theta_star.hpp"

namespace theta_star
{

ThetaStar::ThetaStar()
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

void ThetaStar::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  unsigned int s[2], d[2];
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, s[0], s[1]);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, d[0], d[1]);

  src_ = {static_cast<int>(s[0]), static_cast<int>(s[1])};
  dst_ = {static_cast<int>(d[0]), static_cast<int>(d[1])};
}

// 生成路径入口
bool ThetaStar::generatePath(std::vector<coordsW> & raw_path)
{
  // 初始化全局变量的值
  resetContainers();
  // 初始化节点存储容器
  addToNodesData(index_generated_);
  // 计算起始节点g
  double src_g_cost = getTraversalCost(src_.x, src_.y);
  // 计算起始节点h
  double src_h_cost = getHCost(src_.x, src_.y);
  // 将起始节点的数据信息存放至nodes_data_中
  nodes_data_[index_generated_] =
  {src_.x, src_.y, src_g_cost, src_h_cost, &nodes_data_[index_generated_], true,
    src_g_cost + src_h_cost};
  // 将起始节点加入至优先队列，此时队列中只有起始节点这个数据
  queue_.push({&nodes_data_[index_generated_]});
  // 存储id，将起始节点nodes_data_存放至node_position_中，同时修改其索引，这样全局坐标下，一个坐标会对应一个独立的索引，这样在更新同一个坐标数据时不会再重新开辟内存
  addIndex(src_.x, src_.y, &nodes_data_[index_generated_]);
  // 将起始节点复制到curr_data节点，以后curr_data节点将作为优先级最高的节点去计算，后面的优先队列queue_将会通过top与pop操作不断更新curr_data
  tree_node * curr_data = &nodes_data_[index_generated_];
  // index_generated_是加入至queue_的节点个数
  index_generated_++;
  // nodes_opened是queue_中进行pop操作后剩下存在的节点个数
  nodes_opened = 0;
  // 主循环搜索路径，广度优先搜索
  while (!queue_.empty()) {
    nodes_opened++;
    // 判断是否已经接触到目标点
    if (isGoal(*curr_data)) {
      break;
    }
    // 检查视线（lineofsight）更新父节点，这一步就是与A*的区别，注意is_in_queue，他与后面的setNeighbors函数，以及pop操作联合起来就是将已经使用过的优先队列加入至闭合队列，
    // 以后不会再将curr_data节点重新加入至queue_中
    
    resetParent(curr_data);
    // 计算邻居节点
    setNeighbors(curr_data);
    // 更新curr_data，感觉起始点进行了两次curr_data，因为起始点还没从queue_中pop出去。下一次curr_data就是起始点的邻居了
    curr_data = queue_.top();
    // 这里就直接从队列中删除优先队列，第一次就是起始节点，如果起始节点周围全是障碍物，那queue_就为空了
    queue_.pop();
  }
  // 如果所有点搜索完都没找到goal，说明找不到路径
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

// 检查视线（lineofsight）更新父节点，这是theta*与a*的区别
void ThetaStar::resetParent(tree_node * curr_data)
{
  double g_cost, los_cost = 0;
  // 将当前节点移除开放队列
  curr_data->is_in_queue = false;
  // 当前节点的父节点
  const tree_node * curr_par = curr_data->parent_id;
  // 当前节点父节点的父节点
  const tree_node * maybe_par = curr_par->parent_id;

  // 检查从当前节点到当前节点父节点的父节点视线是否被遮挡，如果没有遮挡则true，可以将爷节点作为父节点，遮挡的话就不更新
  if (losCheck(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y, los_cost)) {
    // 计算成本距离g
    // 成本g除了包含分段距离，还包含每个节点的遍历成本los_cost
    g_cost = maybe_par->g +
      getEuclideanCost(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y) + los_cost;

    // 更新父节点，有时候因为父节点不同，所以计算出的g值也不同，此时选取最小的g对应的父节点作为真正的父节点
    if (g_cost < curr_data->g) {
      curr_data->parent_id = maybe_par;
      curr_data->g = g_cost;
      curr_data->f = g_cost + curr_data->h;
    }
  }
}

// 计算当前节点的8个邻居节点的一些值，同时比较后更新父节点
void ThetaStar::setNeighbors(const tree_node * curr_data)
{
  int mx, my;
  tree_node * m_id = nullptr;
  double g_cost, h_cost, cal_cost;

  for (int i = 0; i < how_many_corners_; i++) {
    mx = curr_data->x + moves[i].x;
    my = curr_data->y + moves[i].y;

    // 检查是否越界，如果越界continue，跳过该节点，返回到for位置计算下个邻居节点，
    // 此时该节点就不会进入node_position_，也就不会进行计算其代价值，也不会将其加入路径
    if (withinLimits(mx, my)) {
      // 检查是都是可通行的节点,如果该节点有障碍物或者未知区域不可取时则跳过该节点
      if (!isSafe(mx, my)) {
        continue;
      }
    } else {
      continue;
    }
    // 这里计算curr_data邻居的g，所以父节点就是curr_data，则g=父节点g+邻居与父节点之间的距离+邻居的代价图的代价值
    g_cost = curr_data->g + getEuclideanCost(curr_data->x, curr_data->y, mx, my) + getTraversalCost(mx, my);
    // 取得节点指针
    m_id = getIndex(mx, my);
    // m_id最开始默认为nullptr，当后面计算越来越多时，可能就不会了
    if (m_id == nullptr) {
      // 插入节点数据，也就是放入nodes_data_中
      addToNodesData(index_generated_);
      m_id = &nodes_data_[index_generated_];
      // 插入节点指针索引，也就是放入至node_position_中
      addIndex(mx, my, m_id);
      index_generated_++;
    }
    // 此时将该节点另存为exp_node，m_id旧的一些值
    exp_node = m_id;
    h_cost = getHCost(mx, my);
    // 这里的cal_cost就是m_id计算出新的f
    cal_cost = g_cost + h_cost;
    // 比较f值判断是否需要更改父节点。这里就比较旧的和新的m_id，因为父节点不同，所以f值可能不同。同时这里不会将以前作为优先队列的节点再重新加入queue_中，这个设计真的很巧妙
    // 这里就获取最小f值与其对应的父节点
    // 注意这里的处理，他是顺便处理了闭合队列的。
    // 比如：起始节点作为优先队列时，在resetParent函数中先将起始节点的is_in_queue=false，然后setNeighbors函数先将8个邻居更新f值，假设8个邻居都是可通行的嘛，那么其中一个就会作为下一个优先队列
    // 如果其中一个邻居（代号new_cur）作为优先队列，就会计算new_cur的8个邻居（假设同样都是可达点），起始点也同样作为new_cur的邻居了，但是起始点不会加入至queue_。因为起始点已经pop出去了，同时exp_node->f > cal_cost这个条件不会满足。但是其他7个邻居
    // 会在queue_中，同时作为起始点与new_cur的邻居，在起始点作为优先队列时就已经将这些共同邻居加入至queue_了，所以这次不管exp_node->f > cal_cost这个条件
    // 所以在queue_中的is_in_queue都是设为true了的。如果要进入闭合队列，就会先从queue_中top+pop操作，然后设置is_in_queue为false，这样以后就不会再将其加入至queue_了。
    if (exp_node->f > cal_cost) {
      exp_node->g = g_cost;
      exp_node->h = h_cost;
      exp_node->f = cal_cost;
      exp_node->parent_id = curr_data;
      // 判断节点是否已经在队列中，不在就加入队列，在的话上一步就更新值就行了
      // 最开始默认的都不在
      if (!exp_node->is_in_queue) {
        exp_node->x = mx;
        exp_node->y = my;
        exp_node->is_in_queue = true;
        queue_.push({m_id});
      }
    }
  }
}

// 依据父节点回溯获取路径
void ThetaStar::backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const
{
  std::vector<coordsW> path_rev;
  coordsW world{};
  // do...while先将当前节点加入至path_rev，然后找父节点，循环加入和找父节点步骤，直到父节点是自己时，也就是到了起点，此时循环结束
  do {
    costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
    path_rev.push_back(world);
    if (path_rev.size() > 1) {
      curr_n = curr_n->parent_id;
    }
  } while (curr_n->parent_id != curr_n);
  // 将起点切换成全局坐标并加入至path_rev
  costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
  path_rev.push_back(world);
  // 路径点进行反向操作保存至raw_points，因为路径点是从目标点开始找回到起点的，我们需要的是从起点到目标点的路径
  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

// 视线检查是否被遮挡
bool ThetaStar::losCheck(
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

// 初始化全局变量的值
void ThetaStar::resetContainers()
{
  index_generated_ = 0;
  int last_size_x = size_x_;
  int last_size_y = size_y_;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (((last_size_x != curr_size_x) || (last_size_y != curr_size_y)) &&
    static_cast<int>(node_position_.size()) < (curr_size_x * curr_size_y))
  {
    // 初始化索引容器
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data_.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
}

// 初始化索引容器
void ThetaStar::initializePosn(int size_inc)
{
  int i = 0;

  if (!node_position_.empty()) {
    for (; i < size_x_ * size_y_; i++) {
      node_position_[i] = nullptr;
    }
  }

  for (; i < size_inc; i++) {
    node_position_.push_back(nullptr);
  }
}
}  //  namespace theta_star
