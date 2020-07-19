#pragma once
#include <vector>
#include "value_equal.h"
#include "logger.h"

/*
*section类代表了stl实体被平行于z轴的平面切割后产生的二维轮廓，该轮廓由首尾相连的线段组成。
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
        SPDLOG_LOGGER_CRITICAL(logger(), "section error.");
        spdlog::shutdown();
        exit(-1);
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

  //点p_index属于两条边：e_index和该函数返回值。
  size_t getTheOtherPointIndex(size_t e_index, size_t p_index);

public:
  using point_type = point;
  using edge_type = edge;
  enum class state { COINCIDENCE, INNER, OUTER, ONBOUNDARY };

  Section();
  //新建一个edge，插入edges_并记录该edge_index，然后p1和p2记录该index并插入points_
  void push_edge(point& p1, point& p2);

  void merge();

  void topo_check();

  void classify(double min, size_t y_count, double grid_size);

  state judge_p2(const point& p1, const point& p2, size_t i);

private:
  std::vector<edge> edges_;
  std::vector<point> points_;
  std::vector<std::vector<size_t>> bucket_;
};
