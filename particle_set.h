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

  ParticleSet() = default;

  void init(size_type x, size_type y, size_type z) {
    if (x <= 0 || y <= 0 || z <= 0) {
      capture.log_fatal(1, piece("particle constructor error! :", x, ",", y, ",", z));
    }
    nodes_.resize(x * y * z);
    current_x_ = current_y_ = current_z_ = 0;
    x_count_ = x;
    y_count_ = y;
    z_count_ = z;
  }

  ParticleSet& operator()(size_type x, size_type y, size_type z) {
    current_x_ = x;
    current_y_ = y;
    current_z_ = z;
    return *this;
  }
  ParticleSet& operator=(const node& n) {
    nodes_.at(get_index(current_x_, current_y_, current_z_)) = n;
    return *this;
  }

  void out_to_tecplot(std::ostream& out) {
    out << "TITLE = \"TEST\"\n";
    out << "VARIABLES =\"X\",\"Y\",\"Z\",\"T\"\n";
    size_type nx = x_count_;
    size_type ny = y_count_;
    size_type nz = z_count_;
    out << "ZONE I=" << nx << ",J=" << ny << ",K=" << nz << ",F=POINT\n";
    for (size_type k = 0; k < nz; ++k) {
      for (size_type j = 0; j < ny; ++j) {
        for (size_type i = 0; i < nx; ++i) {
          size_type offset = get_index(i, j, k);
          out << i << " " << j << " " << k << " " << nodes_.at(offset).type << "\n";
        }
      }
    }
  }
};