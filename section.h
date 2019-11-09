#pragma once
#include <vector>
#include "value_equal.h"
#include "pnlog.h"
/*
*section�������stlʵ�屻ƽ����z���ƽ���и������Ķ�ά����������������β�������߶���ɡ�
*/
class Section {
private:
  struct point {
    double x;
    double y;
    std::vector<size_t> edge_index;
    point(const point& other) = default;
    point(double x_, double y_) :x(x_), y(y_) {

    }
    point() = default;
    bool operator<(const point& other) {
      if (value_s(x, other.x)) {
        return true;
      }
      else if (value_equal(x, other.x) && value_s(y, other.y)) {
        return true;
      }
      return false;
    }
    bool operator==(const point& other) {
      return value_equal(x, other.x) && value_equal(y, other.y);
    }
  };

  struct edge {
    std::vector<size_t> point_index_;
    size_t get_the_other_point_index(size_t p) {
      size_t result;
      if (p == point_index_[0])
        result = point_index_[1];
      else if (p == point_index_[1]) {
        result = point_index_[0];
      }
      else {
        capture.log_fatal(1, piece("section error!"));
      }
      return result;
    }
  };

  template<class T>
  T the_smaller(const T& t1, const T& t2) {
    if (value_s(t1, t2))
      return t1;
    return t2;
  }

  template<class T>
  T the_lager(const T& t1, const T& t2) {
    if (value_b(t1, t2))
      return t1;
    return t2;
  }

  //��p_index���������ߣ�e_index�͸ú�������ֵ��
  size_t getTheOtherPointIndex(size_t e_index, size_t p_index) {
    size_t e1 = points_.at(p_index).edge_index[0];
    size_t e2 = points_.at(p_index).edge_index[1];
    size_t e;
    if (e1 == e_index) {
      e = e2;
    }
    else if (e2 == e_index) {
      e = e1;
    }
    else {
      capture.log_fatal(1, __LINE__, __FILE__, piece("section error!"));
    }
    size_t p1 = edges_.at(e).point_index_[0];
    size_t p2 = edges_.at(e).point_index_[1];
    size_t p;
    if (p1 == p_index) {
      p = p2;
    }
    else if (p2 == p_index) {
      p = p1;
    }
    else {
      capture.log_fatal(1, __LINE__, __FILE__, piece("section error!"));
    }
    return p;
  }

public:
  using point_type = point;
  using edge_type = edge;
  enum class state { COINCIDENCE, INNER, OUTER, ONBOUNDARY };

  Section() = default;
  //�½�һ��edge������edges_����¼��edge_index��Ȼ��p1��p2��¼��index������points_
  void push_edge(point p1, point p2) {
    edge tmp;
    edges_.push_back(tmp);
    p1.edge_index.push_back(edges_.size() - 1);
    p2.edge_index.push_back(edges_.size() - 1);
    points_.push_back(p1);
    points_.push_back(p2);
  }

  void merge() {
    std::vector<point> new_points;
    std::sort(points_.begin(), points_.end());

    std::vector<point>::iterator point_last;
    for (auto it = points_.begin(); it != points_.end(); ++it) {
      if (it == points_.begin()) {
        point_last = it;
      }
      else if (*it == *point_last) {
        for (auto& each : it->edge_index) {
          point_last->edge_index.push_back(each);
        }//�̳��ظ���ı�������Ϣ��
      }
      else {
        new_points.push_back(*point_last);
        point_last = it;
      }
      if (it == points_.end() - 1) {
        new_points.push_back(*point_last);
      }
    }

    points_ = std::move(new_points);

    size_t point_index_ = 0;
    for (auto it = points_.begin(); it != points_.end(); ++it) {
      for (auto it2 = it->edge_index.begin(); it2 != it->edge_index.end(); ++it2) {
        edges_.at(*it2).point_index_.push_back(point_index_);
      }
      ++point_index_;
    }
  }

  void topo_check() {
    for (auto& each : edges_) {
      if (each.point_index_.size() != 2) {
        //LOG_FATAL << "each edge need two points! : " << each.point_index.size();
        capture.log_fatal(1, piece("each section edge need two points! : ", each.point_index_.size()));
      }
    }

    size_t point_index = 0;
    for (auto& each : points_) {
      if (each.edge_index.size() != 2) {
        //LOG_FATAL << "each point belongs two edges! : " << each.edge_index.size() << " " << each.x << " " << each.y << " " << point_index;
        capture.log_fatal(1, piece("each section point belongs two edges ! : ", each.edge_index.size(), " ", each.x, " ", each.y));
      }
      ++point_index;
    }
  }

  void classify(double min, size_t y_count, double grid_size) {
    bucket_.resize(y_count);
    size_t edge_index = 0;
    for (auto it = edges_.begin(); it != edges_.end(); ++it) {
      double y_max = the_lager(points_.at(it->point_index_[0]).y, points_.at(it->point_index_[1]).y);
      double y_min = the_smaller(points_.at(it->point_index_[0]).y, points_.at(it->point_index_[1]).y);

      for (size_t i = 0; i < y_count; ++i) {
        double left = min + i * grid_size;
        double right = left + grid_size;
        if (value_s(y_max, left) || value_b(y_min, right)) {
          continue;
        }
        bucket_.at(i).push_back(edge_index);
      }

      ++edge_index;
    }
  }

  state judge_p2(const point& p1, const point& p2, size_t i) {
    //����p1-p2���߶Σ�����ͳ������ƽ�����߶εĹ�ϵ��
    //���p2���߶��ϣ��򷵻�ONBOUNDARY
    //���p1-p2��ĳ���߶β����غϣ�����COINCIDENCE
    //ͳ�ƴ�͸�߽�Ĵ�����ȷ��p2��p1���ԵĹ�ϵ��������ֱ�Ӵ���һ���߶Σ�Ҳ���ܾ��������ߵĶ˵㣬��ʱ��Ҫ�ж��������߶��Ƿ���ͬ�ࡣ��
    size_t penetration_times = 0;
    size_t penetration_point_times = 0;
    for (auto it = bucket_[i].begin(); it != bucket_[i].end(); ++it) {
      point& ep1 = points_.at(edges_.at(*it).point_index_[0]);
      point& ep2 = points_.at(edges_.at(*it).point_index_[1]);
      if (value_s(ep1.y, p1.y) && value_s(ep2.y, p1.y)) {
        continue;
      }
      else if (value_b(ep1.y, p1.y) && value_b(ep2.y, p1.y)) {
        continue;
      }
      else {
        //��ʱ�����ཻ
        if (value_equal(ep1.y, ep2.y) && value_equal(ep1.y, p1.y)) {
          return  state::COINCIDENCE;
        }
        double x0 = (p1.y - ep1.y) / (ep2.y - ep1.y) * (ep2.x - ep1.x) + ep1.x;
        if (value_b(x0, p2.x) || value_s(x0, p1.x) || value_equal(x0, p1.x)) {
          continue;
        }
        //��ʱ���߶���p1-p2�ཻ��������(p1.x,p2.x]�ϡ�
        else if (value_equal(x0, p2.x)) {
          //��ʱp2�����߶��ϣ�ֱ�ӷ���
          if (value_equal(x0, ep1.x)) {
            //��ʱp0-p1���߶εĽ���պ��Ǹ��߶εĶ˵�
            return state::ONBOUNDARY;
          }
        }
        //��ʱ����һ����(p1.x,p2.x)��
        if (value_equal(ep1.y, p1.y)) {
          //��ʱp0-p1���߶εĽ���պ��Ǹ��߶εĶ˵�
          size_t p = getTheOtherPointIndex(*it, edges_.at(*it).point_index_[0]);
          if (value_s(ep2.y, p1.y) && value_s(points_.at(p).y, p1.y)) {
            continue;
          }
          else if (value_b(ep2.y, p1.y) && value_b(points_.at(p).y, p1.y)) {
            continue;
          }
          else {
            ++penetration_point_times;
          }
        }
        else if (value_equal(ep2.y, p1.y)) {
          //��ʱp0-p1���߶εĽ���պ��Ǹ��߶εĶ˵�
          size_t p = getTheOtherPointIndex(*it, edges_.at(*it).point_index_[1]);
          if (value_s(ep1.y, p1.y) && value_s(points_.at(p).y, p1.y)) {
            continue;
          }
          else if (value_b(ep1.y, p1.y) && value_b(points_.at(p).y, p1.y)) {
            continue;
          }
          else {
            ++penetration_point_times;
          }
        }
        else {
          //��ʱ����򵥵������
          ++penetration_times;
        }
      }
    }
    if (penetration_point_times % 2 != 0) {
      //LOG_FATAL << "ERROR : " << p1.x << " " << p1.y << " " << p2.x << " " << p2.y;
      capture.log_fatal(1, __LINE__, __FILE__, piece("error : ", p1.x, " ", p1.y, " ,", p2.x, " ", p2.y));
    }
    size_t judge = penetration_times + penetration_point_times / 2;
    if (judge % 2 == 0) {
      return state::OUTER;
    }
    return state::INNER;
  }
private:
  std::vector<edge> edges_;
  std::vector<point> points_;
  std::vector<std::vector<size_t>> bucket_;
};