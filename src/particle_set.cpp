#include "particle_set.h"
#include "logger.h"

ParticleSet::ParticleSet() = default;

void ParticleSet::init(size_type x, size_type y, size_type z) {
  if (x <= 0 || y <= 0 || z <= 0) {
    SPDLOG_LOGGER_CRITICAL(logger(), "particle constructor error! : {}, {}, {}", x, y, z);
    spdlog::shutdown();
    exit(-1);
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

void ParticleSet::out_to_sgn(std::ostream& out) {
#pragma pack(1)
  struct head {
    char version[10];
    short flag;
    short nx;
    short ny;
    short nz;
    char space[238];
  };
#pragma pack()
  head head_;
  head_.flag = -1;
  head_.nx = x_count_;
  head_.ny = y_count_;
  head_.nz = z_count_;
  out.write((char*)&head_, sizeof(head_));
  size_type node_count = head_.nx * head_.ny * head_.nz;
  short* sgn = new short[node_count];

  for (size_type k = 0; k < head_.nz; ++k) {
    for (size_type j = 0; j < head_.ny; ++j) {
      for (size_type i = 0; i < head_.nx; ++i) {
        size_type offset = get_index(i, j, k);
        if (nodes_[offset].type == 0 || nodes_[offset].type == 1) {
          sgn[offset] = 0;
        }
        else {
          sgn[offset] = 2;
        }
      }
    }
  }
  out.write((char*)sgn, sizeof(short) * node_count);
  float *dx = new float[head_.nx];
  for (size_t i = 0; i < head_.nx; ++i) {
    dx[i] = grid_size_;
  }
  out.write((char*)dx, sizeof(float) * head_.nx);
  delete[] dx;

  float *dy = new float[head_.ny];
  for (size_t i = 0; i < head_.ny; ++i) {
    dy[i] = grid_size_;
  }
  out.write((char*)dy, sizeof(float) * head_.ny);
  delete[] dy;

  float *dz = new float[head_.nz];
  for (size_type i = 0; i < head_.nz; ++i) {
    dz[i] = grid_size_;
  }
  out.write((char*)dz, sizeof(float) * head_.nz);
  delete[] dz;
}
