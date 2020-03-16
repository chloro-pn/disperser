#pragma once
#include "value_equal.h"
#include <vector>
#include <algorithm>

class BoundaryNode {
public:
  struct node {
    double x_;
    double y_;
    double z_;
    double norm_[3];

    bool operator<(const node& other) const {
      if (x_ < other.x_) {
        return true;
      }
      if (x_ == other.x_ && y_ < other.y_) {
        return true;
      }
      if (x_ == other.x_ && y_ == other.y_ && z_ < other.z_) {
        return true;
      }
      return false;
    }

    bool operator==(const node& other) const {
      return value_equal(x_, other.x_) && value_equal(y_, other.y_) && value_equal(z_, other.z_);
    }
  };

  BoundaryNode() = default;
  void push_node(const node& n) {
    b_nodes_.push_back(n);
  }

  void delete_repeat() {
    std::sort(b_nodes_.begin(), b_nodes_.end());
    for (auto it = b_nodes_.begin(); it != b_nodes_.end();) {
      auto next = it + 1;
      if (next == b_nodes_.end()) {
        break;
      }
      if (*it == *next) {
        it = b_nodes_.erase(it);
      }
      else {
        ++it;
      }
    }
  }

private:
  std::vector<node> b_nodes_;
};
