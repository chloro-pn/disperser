#pragma once
#include <vector>
#include <ostream>
#include "type.h"
#include "pnlog.h"

class ParticleSet {
private:
  /*
  *type == 0 : 内部点
  *type == 1 : 边界点
  *type == 2 : 外部点
  */
  struct node {
    double x;
    double y;
    double z;
    int type;
    node() = default;
    node(double xx,double yy,double zz,int t):x(xx),y(yy),z(zz),type(t) {}
    node& operator=(const node& other) = default;
  };

  std::vector<node> nodes_;
  size_type x_count_;
  size_type y_count_;
  size_type z_count_;

  size_type current_x_;
  size_type current_y_;
  size_type current_z_;

  size_type get_index(size_type x, size_type y, size_type z) const {
    if (x < 0 || y < 0 || z < 0 || x >= x_count_ || y >= y_count_ || z >= z_count_) {
      return -1;
    }
    return z + z_count_ * y + y_count_ * z_count_ * x;
  }

public:
  using node_type = node;

  ParticleSet();

  void init(size_type x, size_type y, size_type z);

  ParticleSet& operator()(size_type x, size_type y, size_type z);

  ParticleSet& operator=(const node& n);

  void out_to_tecplot(std::ostream& out);
};