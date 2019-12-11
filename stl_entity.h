#pragma once
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include "value_equal.h"
#include "pnlog.h"
#include "section.h"
#include "particle_set.h"
#include "boundary_node.h"

/*
*类StlEntity表示stl三维文件在内存中的重构。
*作用1：从磁盘导入stl三维文件
*作用2：对文件进行拓扑检查
*作用3：对文件进行包围盒构造
*/
class StlEntity {
private:
  /*
  *结构triangle_表示一个三角形面片在二进制stl文件中的存储结构。
  */
#pragma pack(1)
  struct triangle_ {
    float normal[3];
    float first[3];
    float second[3];
    float third[3];
    char buf[2];

    triangle_& operator=(const triangle_& other) {
      normal[0] = other.normal[0];
      normal[1] = other.normal[1];
      normal[2] = other.normal[2];
      first[0] = other.first[0];
      first[1] = other.first[1];
      first[2] = other.first[2];
      second[0] = other.second[0];
      second[1] = other.second[1];
      second[2] = other.second[2];
      third[0] = other.third[0];
      third[1] = other.third[1];
      third[2] = other.third[2];
      return *this;
    }
  };
#pragma pack()
  /*
  *结构triangle表示一个三角形面片在内存中的存储结构
  */
  struct triangle {
    std::array<float, 3> normal;
    std::array<float, 3> first;
    std::array<float, 3>second;
    std::array<float, 3> third;

    triangle& operator=(const triangle_& other) {
      normal[0] = other.normal[0];
      normal[1] = other.normal[1];
      normal[2] = other.normal[2];
      first[0] = other.first[0];
      first[1] = other.first[1];
      first[2] = other.first[2];
      second[0] = other.second[0];
      second[1] = other.second[1];
      second[2] = other.second[2];
      third[0] = other.third[0];
      third[1] = other.third[1];
      third[2] = other.third[2];
      return *this;
    }
  };

  //StlEntity对导入的三角形面片进行重组，合并重合点和边，用面集合、点集合和边集合表示其拓扑结构
  /*
  *结构tria表示一个三角形面片在内存中的拓扑表示
  */
  struct tria {
    float normal[3];
    std::vector<size_t> point_index_;
    std::vector<size_t> edge_index_;
  };

  /*
  *结构point表示一个点在内存中的拓扑表示
  */
  struct point {
    double x;
    double y;
    double z;
    std::vector<size_t> tria_index_;
    std::vector<size_t> edge_index_;
    point() = default;
    //==语义只比较x、y和z的值，而不比较面集合和边集合
    bool operator==(const point& other) const {
      return (value_equal(x, other.x) && value_equal(y, other.y) && value_equal(z, other.z));
    }

    bool operator<(const point& other) const {
      if (x < other.x) {
        return true;
      }
      if (x == other.x && y < other.y) {
        return true;
      }
      if (x == other.x && y == other.y && z < other.z) {
        return true;
      }
      return false;
    }

    point(const point& other) = default;
    point& operator=(const point& other) = default;
  };

  /*
  结构edge表示一条边在内存中的拓扑表示
  */
  struct edge {
    size_t point_one;
    size_t point_two;
    std::vector<size_t> tria_index_;
    bool operator<(const edge& other) {
      if (point_one < other.point_one) {
        return true;
      }
      if (point_one == other.point_one && point_two < other.point_two) {
        return true;
      }
      return false;
    }
  };

  /*
  *结构InclusionBox表示该stl实体的包围盒
  */
#define MaxUpdate(x1,x2,x3,y) \
  if (x1 > y) { \
    y = x1; \
  } \
  if (x2 > y) { \
    y = x2; \
  } \
  if (x3 > y) {\
    y = x3; \
  }

#define MinUpdate(x1,x2,x3,y) \
  if (x1 < y) { \
    y = x1; \
  } \
  if (x2 < y) { \
    y = x2; \
  } \
  if (x3 < y) {\
    y = x3; \
  }
  struct InclusionBox {
    double least_x;
    double least_y;
    double least_z;
    double max_x;
    double max_y;
    double max_z;

    InclusionBox() = default;
    explicit InclusionBox(triangle_& tmp) {
      max_x = least_x = tmp.first[0];
      max_y = least_y = tmp.first[1];
      max_z = least_z = tmp.first[2];
      update(tmp);
    }
    InclusionBox& operator=(const InclusionBox& other) {
      least_x = other.least_x;
      least_y = other.least_y;
      least_z = other.least_z;
      max_x = other.max_x;
      max_y = other.max_y;
      max_z = other.max_z;
      return *this;
    }
    void init(const triangle_& tmp) {
      max_x = least_x = tmp.first[0];
      max_y = least_y = tmp.first[1];
      max_z = least_z = tmp.first[2];
      update(tmp);
    }
    void update(const triangle_& tmp) {
      MaxUpdate(tmp.first[0], tmp.second[0], tmp.third[0], max_x);
      MaxUpdate(tmp.first[1], tmp.second[1], tmp.third[1], max_y);
      MaxUpdate(tmp.first[2], tmp.second[2], tmp.third[2], max_z);
      MinUpdate(tmp.first[0], tmp.second[0], tmp.third[0], least_x);
      MinUpdate(tmp.first[1], tmp.second[1], tmp.third[1], least_y);
      MinUpdate(tmp.first[2], tmp.second[2], tmp.third[2], least_z);
    }
  };

  struct TdVector {
    double x_;
    double y_;
    double z_;
    TdVector(const TdVector& other) :x_(other.x_), y_(other.y_), z_(other.z_) {}
    TdVector(double x, double y, double z) :x_(x), y_(y), z_(z) {}
    double operator*(const TdVector& other) {
      return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
    }
    TdVector cross(const TdVector& other) {
      return TdVector(y_ * other.z_ - other.y_ * z_, z_ * other.x_ - other.z_ * x_, x_ * other.y_ - other.x_ * y_);
    }
  };

public:
  StlEntity();

  void load(std::string filename);

  void disperse(double grid_size);

  std::vector<size_t> try_disperse(double grid_size);

  void out_to_tecplot(std::string filename);

  void out_to_sgn(std::string filename);

//工具函数
private:
  //如果当前y_liner处于特殊位置，则需要进行微调（例如有二维截面上的线段刚好落在该直线上）
  double _get_real_y_liner_(double yl, size_t x_count, InclusionBox& box, double grid_size, Section& z_secion, size_t j);
  size_t _get_other_point_on_other_tria(size_t tria_index, size_t edge_index);
  //如果当前z_plane处于特殊位置，需要进行微调（例如有三角面刚好落在该平面上）
  double _get_real_z_plane_(double zp);
  size_t get_edge_index_from_two_point(size_t p1, size_t p2);
  //将三角面片按照分布区间分别放入三个维度的桶里。
  void _tri_classify_(double x_min, size_t x_count, double y_min, size_t y_count, double z_min, size_t z_count, double grid_size);
  //result[0].x >= result[1].x >= result[2].x
  std::vector<point> _order_by_x_(const point& p1, const point& p2, const point& p3);
  std::vector<point> _order_by_x_(size_t tria_index);
  std::vector<point> _order_by_y_(const point& p1, const point& p2, const point& p3);
  std::vector<point> _order_by_y_(size_t tria_index);
  std::vector<point> _order_by_z_(const point& p1, const point& p2, const point& p3);
  std::vector<point> _order_by_z_(size_t tria_index);
  InclusionBox _get_inclusion_box_from_grid_size_(double grid_size);
  void _create_tria_set_and_point_set_();
  void _merge_point_set_();
  void _set_point_index_of_tria_set_();
  void _create_edge_set_();
  void _merge_edge_set_();
  void _set_edge_index_for_tria_and_point_set_();
  void _find_boundary_node_();
  BoundaryNode::node _find_boundary_(const ParticleSet::node& p1, const ParticleSet::node& p2, size_type x, size_type y, size_type z);
  std::vector<double> get_average_norm(std::vector<size_t> tri_index_);
  void init_topo();
  void topo_check();

private:
  bool loaded_;
  char information_[80];
  uint32_t numbers_; //stl file中用四个字节存储三角面片个数.
  std::vector<triangle> triangles_;
  InclusionBox box_;
  /*以下是拓扑结构*/
  std::vector<point> point_set_;
  std::vector<tria> tria_set_;
  std::vector<edge> edge_set_;
  /*以下是对三角形集合的桶分类*/
  std::vector<std::vector<size_t>> tri_bucket_x_;
  std::vector<std::vector<size_t>> tri_bucket_y_;
  std::vector<std::vector<size_t>> tri_bucket_z_;
  /*以下是存储结果类*/
  ParticleSet ps_;
  BoundaryNode bd_;
  bool get_result_;
  /*工具函数*/
  template<class T>
  inline T the_smaller(const T& t1, const T& t2) {
    return t1 < t2 ? t1 : t2;
  }

  template<class T>
  inline T the_bigger(const T& t1, const T& t2) {
    return t1 > t2 ? t1 : t2;
  }
};