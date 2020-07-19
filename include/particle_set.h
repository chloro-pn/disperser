#pragma once
#include <vector>
#include <ostream>
#include <cstdlib>
#include "logger.h"
#include "type.h"

class ParticleSet {
public:
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

private:
  std::vector<node> nodes_;
  size_type x_count_;
  size_type y_count_;
  size_type z_count_;

  size_type current_x_;
  size_type current_y_;
  size_type current_z_;

  double grid_size_;

public:
  using node_type = node;

  size_type x_count() const {
    return x_count_;
  }

  size_type y_count() const {
    return y_count_;
  }

  size_type z_count() const {
    return z_count_;
  }

  const node& get_node(size_type i, size_type j, size_type k) const {
    size_type index = get_index(i, j, k);
    if (index == -1) {
      SPDLOG_LOGGER_CRITICAL(logger(), "node : {} {} {} out of range.", i, j, k);
      spdlog::shutdown();
      exit(-1);
    }
    return nodes_.at(index);
  }

  ParticleSet();

  void init(size_type x, size_type y, size_type z);

  ParticleSet& operator()(size_type x, size_type y, size_type z);

  ParticleSet& operator=(const node& n);

  void out_to_tecplot(std::ostream& out);

  void out_to_sgn(std::ostream& out);

  void set_grid_size(double size) {
    grid_size_ = size;
  }

  size_type get_index(size_type x, size_type y, size_type z) const {
    if (x < 0 || y < 0 || z < 0 || x >= x_count_ || y >= y_count_ || z >= z_count_) {
      return -1;
    }
    return z + z_count_ * y + y_count_ * z_count_ * x;
  }

};
