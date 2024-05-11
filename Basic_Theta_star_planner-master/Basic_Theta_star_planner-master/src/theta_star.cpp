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

// ����·�����
bool ThetaStar::generatePath(std::vector<coordsW> & raw_path)
{
  // ��ʼ��ȫ�ֱ�����ֵ
  resetContainers();
  // ��ʼ���ڵ�洢����
  addToNodesData(index_generated_);
  // ������ʼ�ڵ�g
  double src_g_cost = getTraversalCost(src_.x, src_.y);
  // ������ʼ�ڵ�h
  double src_h_cost = getHCost(src_.x, src_.y);
  // ����ʼ�ڵ��������Ϣ�����nodes_data_��
  nodes_data_[index_generated_] =
  {src_.x, src_.y, src_g_cost, src_h_cost, &nodes_data_[index_generated_], true,
    src_g_cost + src_h_cost};
  // ����ʼ�ڵ���������ȶ��У���ʱ������ֻ����ʼ�ڵ��������
  queue_.push({&nodes_data_[index_generated_]});
  // �洢id������ʼ�ڵ�nodes_data_�����node_position_�У�ͬʱ�޸�������������ȫ�������£�һ��������Ӧһ�������������������ڸ���ͬһ����������ʱ���������¿����ڴ�
  addIndex(src_.x, src_.y, &nodes_data_[index_generated_]);
  // ����ʼ�ڵ㸴�Ƶ�curr_data�ڵ㣬�Ժ�curr_data�ڵ㽫��Ϊ���ȼ���ߵĽڵ�ȥ���㣬��������ȶ���queue_����ͨ��top��pop�������ϸ���curr_data
  tree_node * curr_data = &nodes_data_[index_generated_];
  // index_generated_�Ǽ�����queue_�Ľڵ����
  index_generated_++;
  // nodes_opened��queue_�н���pop������ʣ�´��ڵĽڵ����
  nodes_opened = 0;
  // ��ѭ������·���������������
  while (!queue_.empty()) {
    nodes_opened++;
    // �ж��Ƿ��Ѿ��Ӵ���Ŀ���
    if (isGoal(*curr_data)) {
      break;
    }
    // ������ߣ�lineofsight�����¸��ڵ㣬��һ��������A*������ע��is_in_queue����������setNeighbors�������Լ�pop���������������ǽ��Ѿ�ʹ�ù������ȶ��м������պ϶��У�
    // �Ժ󲻻��ٽ�curr_data�ڵ����¼�����queue_��
    
    resetParent(curr_data);
    // �����ھӽڵ�
    setNeighbors(curr_data);
    // ����curr_data���о���ʼ�����������curr_data����Ϊ��ʼ�㻹û��queue_��pop��ȥ����һ��curr_data������ʼ����ھ���
    curr_data = queue_.top();
    // �����ֱ�ӴӶ�����ɾ�����ȶ��У���һ�ξ�����ʼ�ڵ㣬�����ʼ�ڵ���Χȫ���ϰ����queue_��Ϊ����
    queue_.pop();
  }
  // ������е������궼û�ҵ�goal��˵���Ҳ���·��
  if (queue_.empty()) {
    raw_path.clear();
    return false;
  }
  // ���ݸ��ڵ���ݻ�ȡ·��
  backtrace(raw_path, curr_data);
  // ÿ��ִ�����֮��������ȶ���
  clearQueue();
  return true;
}

// ������ߣ�lineofsight�����¸��ڵ㣬����theta*��a*������
void ThetaStar::resetParent(tree_node * curr_data)
{
  double g_cost, los_cost = 0;
  // ����ǰ�ڵ��Ƴ����Ŷ���
  curr_data->is_in_queue = false;
  // ��ǰ�ڵ�ĸ��ڵ�
  const tree_node * curr_par = curr_data->parent_id;
  // ��ǰ�ڵ㸸�ڵ�ĸ��ڵ�
  const tree_node * maybe_par = curr_par->parent_id;

  // ���ӵ�ǰ�ڵ㵽��ǰ�ڵ㸸�ڵ�ĸ��ڵ������Ƿ��ڵ������û���ڵ���true�����Խ�ү�ڵ���Ϊ���ڵ㣬�ڵ��Ļ��Ͳ�����
  if (losCheck(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y, los_cost)) {
    // ����ɱ�����g
    // �ɱ�g���˰����ֶξ��룬������ÿ���ڵ�ı����ɱ�los_cost
    g_cost = maybe_par->g +
      getEuclideanCost(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y) + los_cost;

    // ���¸��ڵ㣬��ʱ����Ϊ���ڵ㲻ͬ�����Լ������gֵҲ��ͬ����ʱѡȡ��С��g��Ӧ�ĸ��ڵ���Ϊ�����ĸ��ڵ�
    if (g_cost < curr_data->g) {
      curr_data->parent_id = maybe_par;
      curr_data->g = g_cost;
      curr_data->f = g_cost + curr_data->h;
    }
  }
}

// ���㵱ǰ�ڵ��8���ھӽڵ��һЩֵ��ͬʱ�ȽϺ���¸��ڵ�
void ThetaStar::setNeighbors(const tree_node * curr_data)
{
  int mx, my;
  tree_node * m_id = nullptr;
  double g_cost, h_cost, cal_cost;

  for (int i = 0; i < how_many_corners_; i++) {
    mx = curr_data->x + moves[i].x;
    my = curr_data->y + moves[i].y;

    // ����Ƿ�Խ�磬���Խ��continue�������ýڵ㣬���ص�forλ�ü����¸��ھӽڵ㣬
    // ��ʱ�ýڵ�Ͳ������node_position_��Ҳ�Ͳ�����м��������ֵ��Ҳ���Ὣ�����·��
    if (withinLimits(mx, my)) {
      // ����Ƕ��ǿ�ͨ�еĽڵ�,����ýڵ����ϰ������δ֪���򲻿�ȡʱ�������ýڵ�
      if (!isSafe(mx, my)) {
        continue;
      }
    } else {
      continue;
    }
    // �������curr_data�ھӵ�g�����Ը��ڵ����curr_data����g=���ڵ�g+�ھ��븸�ڵ�֮��ľ���+�ھӵĴ���ͼ�Ĵ���ֵ
    g_cost = curr_data->g + getEuclideanCost(curr_data->x, curr_data->y, mx, my) + getTraversalCost(mx, my);
    // ȡ�ýڵ�ָ��
    m_id = getIndex(mx, my);
    // m_id�ʼĬ��Ϊnullptr�����������Խ��Խ��ʱ�����ܾͲ�����
    if (m_id == nullptr) {
      // ����ڵ����ݣ�Ҳ���Ƿ���nodes_data_��
      addToNodesData(index_generated_);
      m_id = &nodes_data_[index_generated_];
      // ����ڵ�ָ��������Ҳ���Ƿ�����node_position_��
      addIndex(mx, my, m_id);
      index_generated_++;
    }
    // ��ʱ���ýڵ����Ϊexp_node��m_id�ɵ�һЩֵ
    exp_node = m_id;
    h_cost = getHCost(mx, my);
    // �����cal_cost����m_id������µ�f
    cal_cost = g_cost + h_cost;
    // �Ƚ�fֵ�ж��Ƿ���Ҫ���ĸ��ڵ㡣����ͱȽϾɵĺ��µ�m_id����Ϊ���ڵ㲻ͬ������fֵ���ܲ�ͬ��ͬʱ���ﲻ�Ὣ��ǰ��Ϊ���ȶ��еĽڵ������¼���queue_�У���������ĺ�����
    // ����ͻ�ȡ��Сfֵ�����Ӧ�ĸ��ڵ�
    // ע������Ĵ�������˳�㴦���˱պ϶��еġ�
    // ���磺��ʼ�ڵ���Ϊ���ȶ���ʱ����resetParent�������Ƚ���ʼ�ڵ��is_in_queue=false��Ȼ��setNeighbors�����Ƚ�8���ھӸ���fֵ������8���ھӶ��ǿ�ͨ�е����ô����һ���ͻ���Ϊ��һ�����ȶ���
    // �������һ���ھӣ�����new_cur����Ϊ���ȶ��У��ͻ����new_cur��8���ھӣ�����ͬ�����ǿɴ�㣩����ʼ��Ҳͬ����Ϊnew_cur���ھ��ˣ�������ʼ�㲻�������queue_����Ϊ��ʼ���Ѿ�pop��ȥ�ˣ�ͬʱexp_node->f > cal_cost��������������㡣��������7���ھ�
    // ����queue_�У�ͬʱ��Ϊ��ʼ����new_cur���ھӣ�����ʼ����Ϊ���ȶ���ʱ���Ѿ�����Щ��ͬ�ھӼ�����queue_�ˣ�������β���exp_node->f > cal_cost�������
    // ������queue_�е�is_in_queue������Ϊtrue�˵ġ����Ҫ����պ϶��У��ͻ��ȴ�queue_��top+pop������Ȼ������is_in_queueΪfalse�������Ժ�Ͳ����ٽ��������queue_�ˡ�
    if (exp_node->f > cal_cost) {
      exp_node->g = g_cost;
      exp_node->h = h_cost;
      exp_node->f = cal_cost;
      exp_node->parent_id = curr_data;
      // �жϽڵ��Ƿ��Ѿ��ڶ����У����ھͼ�����У��ڵĻ���һ���͸���ֵ������
      // �ʼĬ�ϵĶ�����
      if (!exp_node->is_in_queue) {
        exp_node->x = mx;
        exp_node->y = my;
        exp_node->is_in_queue = true;
        queue_.push({m_id});
      }
    }
  }
}

// ���ݸ��ڵ���ݻ�ȡ·��
void ThetaStar::backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const
{
  std::vector<coordsW> path_rev;
  coordsW world{};
  // do...while�Ƚ���ǰ�ڵ������path_rev��Ȼ���Ҹ��ڵ㣬ѭ��������Ҹ��ڵ㲽�裬ֱ�����ڵ����Լ�ʱ��Ҳ���ǵ�����㣬��ʱѭ������
  do {
    costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
    path_rev.push_back(world);
    if (path_rev.size() > 1) {
      curr_n = curr_n->parent_id;
    }
  } while (curr_n->parent_id != curr_n);
  // ������л���ȫ�����겢������path_rev
  costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
  path_rev.push_back(world);
  // ·������з������������raw_points����Ϊ·�����Ǵ�Ŀ��㿪ʼ�һص����ģ�������Ҫ���Ǵ���㵽Ŀ����·��
  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

// ���߼���Ƿ��ڵ�
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

// ��ʼ��ȫ�ֱ�����ֵ
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
    // ��ʼ����������
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data_.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
}

// ��ʼ����������
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
