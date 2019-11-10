#include "particle_set.h"

ParticleSet::ParticleSet() = default;

void ParticleSet::init(size_type x, size_type y, size_type z) {
  if (x <= 0 || y <= 0 || z <= 0) {
    capture.log_fatal(1, piece("particle constructor error! :", x, ",", y, ",", z));
  }
  nodes_.resize(x * y * z);
  current_x_ = current_y_ = current_z_ = 0;
  x_count_ = x;
  y_count_ = y;
  z_count_ = z;
}

ParticleSet& ParticleSet::operator()(size_type x, size_type y, size_type z) {
  current_x_ = x;
  current_y_ = y;
  current_z_ = z;
  return *this;
}
ParticleSet& ParticleSet::operator=(const node& n) {
  nodes_.at(get_index(current_x_, current_y_, current_z_)) = n;
  return *this;
}

void ParticleSet::out_to_tecplot(std::ostream& out) {
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
        out << i << " " << j << " " << k << " ";
        out << (nodes_.at(offset).type == 1 ? 0 : nodes_.at(offset).type) << "\n";
      }
    }
  }
}