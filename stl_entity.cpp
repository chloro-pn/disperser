#include "stl_entity.h"

StlEntity::StlEntity():loaded_(false), get_result_(false) {}

void StlEntity::load(std::string filename) {
  std::ifstream in(filename, std::ios::binary);
  if (!in.good()) {
    capture.log_fatal(2, piece("file open error : ", filename.c_str()));
  }
  in.read(information_, 80);
  in.read((char*)&numbers_, 4);

  triangle_ tmp;
  triangle tri;
  for (size_t i = 0; i < numbers_; ++i) {
    in.read((char*)&tmp, sizeof(tmp));
    if (in.good() == false) {
      capture.log_fatal(2, __LINE__, __FILE__, piece("file read error!"));
    }
    tri = tmp;
    triangles_.push_back(tri);
    //包围盒更新
    if (i == 0) {
      box_.init(tmp);
    }
    else {
      box_.update(tmp);
    }
  }

  capture.log_debug(2, piece("inclusion box : ", box_.least_x, " ", box_.least_y,
    " ", box_.least_z, " ", box_.max_x, " ", box_.max_y, " ", box_.max_z));

  init_topo();
  loaded_ = true;
}

void StlEntity::init_topo() {
  //1.遍历三角面片，构造三角面片索引表
  //构造点索引表，记录每个点所在的三角面片索引
  _create_tria_set_and_point_set_();
  //合并相同点，并继承所属三角面片信息
  _merge_point_set_();
  //此时节点合并完成，且每个节点所属于的三角面片的索引更新完成。
  //为每个三角面片设置顶点索引
  _set_point_index_of_tria_set_();
  //构建边索引集合
  _create_edge_set_();
  //合并重复边
  _merge_edge_set_();
  //给每个三角面和点设置边索引
  _set_edge_index_for_tria_and_point_set_();
  //拓扑检查
  topo_check();
}

void StlEntity::topo_check() {
  //1.每个三角面应该有三条边和三个点
  size_t tria_index = 0;
  for (auto it = tria_set_.begin(); it != tria_set_.end(); ++it) {
    if (it->edge_index_.size() != 3) {
      capture.log_fatal(2, piece("tria_index : ", tria_index, " , edge size : ", it->edge_index_.size()));
    }
    if (it->point_index_.size() != 3) {
      capture.log_fatal(2, piece("tria_index : ", tria_index, " , point size : ", it->point_index_.size()));
    }
    ++tria_index;
  }

  //2.每条边应该属于两个三角面
  size_t edge_index = 0;
  for (auto it = edge_set_.begin(); it != edge_set_.end(); ++it) {
    if (it->tria_index_.size() != 2) {
      //LOG_FATAL << "edge_index :  " << edge_index << " tria size :  " << it->tria_index_.size();
      capture.log_fatal(2, piece("edge_index : ", edge_index, " , tria size : ", it->tria_index_.size()));
    }
    ++edge_index;
  }
}

void StlEntity::disperse(double grid_size) {
  if (grid_size <= 0) {
    capture.log_fatal(2, piece("grid_size invalid : ", grid_size));
  }
  //每次划分，需要获得一个新的边界盒，原先的边界盒紧贴stl实体，容易产生误差。
  InclusionBox box = _get_inclusion_box_from_grid_size_(grid_size);
  size_t x_count = static_cast<size_t>((box.max_x - box.least_x) / grid_size) + 1;
  size_t y_count = static_cast<size_t>((box.max_y - box.least_y) / grid_size) + 1;
  size_t z_count = static_cast<size_t>((box.max_z - box.least_z) / grid_size) + 1;

  _tri_classify_(box.least_x, x_count, box.least_y, y_count, box.least_z, z_count, grid_size);

  //类ParticleSet存储剖分结果。
  ps_.init(x_count, y_count, z_count);
  using node = ParticleSet::node_type;

  for (size_t k = 0; k < z_count; ++k) {
    double z_plane = box.least_z + k * grid_size;
    z_plane = _get_real_z_plane_(z_plane);
    Section z_section;
    size_t tria_index = 0;
    for (auto it = tria_set_.begin(); it != tria_set_.end(); ++it) {
      point& p1 = point_set_[it->point_index_[0]];
      point& p2 = point_set_[it->point_index_[1]];
      point& p3 = point_set_[it->point_index_[2]];
      std::vector<point> re = _order_by_z_(p1, p2, p3);
      //该三角面片与z_plane平面不相交。
      if (value_s(re[0].z, z_plane) || value_b(re[2].z, z_plane)) {
        ++tria_index;
        continue;
      }
      else if (value_b(re[0].z, z_plane) && value_s(re[2].z, z_plane)) {
        if (value_equal(re[1].z, z_plane) == true) {
          //情况1
          Section::point_type p_tmp_1(re[1].x, re[1].y);
          Section::point_type p_tmp_2;
          p_tmp_2.x = (z_plane - re[2].z) / (re[0].z - re[2].z) * (re[0].x - re[2].x) + re[2].x;
          p_tmp_2.y = (z_plane - re[2].z) / (re[0].z - re[2].z) * (re[0].y - re[2].y) + re[2].y;
          z_section.push_edge(p_tmp_1, p_tmp_2);
          capture.log_debug(2, piece("情况2 ： ", p_tmp_1.x, " ", p_tmp_1.y, ", ", p_tmp_2.x, " ", p_tmp_2.y));
        }
        else if (value_b(re[1].z, z_plane)) {
          Section::point_type p_tmp_1;
          p_tmp_1.x = (z_plane - re[2].z) / (re[0].z - re[2].z) * (re[0].x - re[2].x) + re[2].x;
          p_tmp_1.y = (z_plane - re[2].z) / (re[0].z - re[2].z) * (re[0].y - re[2].y) + re[2].y;

          Section::point_type p_tmp_2;
          p_tmp_2.x = (z_plane - re[2].z) / (re[1].z - re[2].z) * (re[1].x - re[2].x) + re[2].x;
          p_tmp_2.y = (z_plane - re[2].z) / (re[1].z - re[2].z) * (re[1].y - re[2].y) + re[2].y;
          z_section.push_edge(p_tmp_1, p_tmp_2);
          capture.log_debug(2, piece("情况2 ： ", p_tmp_1.x, " ", p_tmp_1.y, ", ", p_tmp_2.x, " ", p_tmp_2.y));
        }
        else {
          Section::point_type p_tmp_1;
          p_tmp_1.x = (z_plane - re[2].z) / (re[0].z - re[2].z) * (re[0].x - re[2].x) + re[2].x;
          p_tmp_1.y = (z_plane - re[2].z) / (re[0].z - re[2].z) * (re[0].y - re[2].y) + re[2].y;

          Section::point_type p_tmp_2;
          p_tmp_2.x = (z_plane - re[1].z) / (re[0].z - re[1].z) * (re[0].x - re[1].x) + re[1].x;
          p_tmp_2.y = (z_plane - re[1].z) / (re[0].z - re[1].z) * (re[0].y - re[1].y) + re[1].y;
          z_section.push_edge(p_tmp_1, p_tmp_2);
          capture.log_debug(2, piece("情况2 ： ", p_tmp_1.x, " ", p_tmp_1.y, ", ", p_tmp_2.x, " ", p_tmp_2.y));
        }
      }
      else if (value_b(re[0].z, z_plane) && value_equal(re[1].z, z_plane) && value_equal(re[2].z, z_plane)) {
        size_t t1 = it->edge_index_[0];
        size_t t2 = it->edge_index_[1];
        size_t t3 = it->edge_index_[2];
        size_t edge_t = t1;
        if (point_set_.at(edge_set_.at(t2).point_one) == re[1] && point_set_.at(edge_set_.at(t2).point_two) == re[2] ||
          point_set_.at(edge_set_.at(t2).point_one) == re[2] && point_set_.at(edge_set_.at(t2).point_two) == re[1]) {
          edge_t = t2;
        }
        else if (point_set_.at(edge_set_.at(t3).point_one) == re[1] && point_set_.at(edge_set_.at(t3).point_two) == re[2] ||
          point_set_.at(edge_set_.at(t3).point_one) == re[2] && point_set_.at(edge_set_.at(t3).point_two) == re[1]) {
          edge_t = t3;
        }
        //edge_t的两个点在平面上
        size_t tmp = _get_other_point_on_other_tria(tria_index, edge_t);
        if (value_b(point_set_.at(tmp).z, z_plane)) {
          //该点也在z_plane面之上.
          ++tria_index;
          continue;
        }
        else {
          Section::point_type p_tmp_1;
          p_tmp_1.x = point_set_.at(edge_set_.at(edge_t).point_one).x;
          p_tmp_1.y = point_set_.at(edge_set_.at(edge_t).point_one).y;

          Section::point_type p_tmp_2;
          p_tmp_2.x = point_set_.at(edge_set_.at(edge_t).point_two).x;
          p_tmp_2.y = point_set_.at(edge_set_.at(edge_t).point_two).y;
          z_section.push_edge(p_tmp_1, p_tmp_2);
          capture.log_debug(2, piece("情况3 : ", p_tmp_1.x, " ", p_tmp_1.y, ", ", p_tmp_2.x, " ", p_tmp_2.y));
        }
      }
      else if (value_equal(re[0].z, z_plane) && value_equal(re[1].z, z_plane) && value_s(re[2].z, z_plane)) {
        //情况4已经被情况3包含，故跳过.
        ++tria_index;
        continue;
      }
      else if (value_equal(re[0].z, z_plane) && value_s(re[1].z, z_plane)) {
        //只有一个点相交，跳过
        ++tria_index;
        continue;
      }
      else if (value_equal(re[2].z, z_plane) && value_b(re[1].z, z_plane)) {
        //只有一个点相交，跳过
        ++tria_index;
        continue;
      }
      else {
        capture.log_fatal(2, piece("error ! : ", z_plane, " ", re[0].z, " ", re[1].z, " ", re[2].z));
      }
      ++tria_index;
    }
    z_section.merge();
    z_section.topo_check();
    z_section.classify(box.least_y, y_count, grid_size);
    //z_secion中的二维截面图已经构造好。
    for (size_t j = 0; j < y_count; ++j) {
      double y_liner = box.least_y + j * grid_size;
      y_liner = _get_real_y_liner_(y_liner, x_count, box, grid_size, z_section, j);
      for (size_t i = 0; i < x_count; ++i) {
        double x_begin = box.least_x;
        double x_point = box.least_x + i * grid_size;
        if (i == 0) {
          ps_(i, j, k) = node(x_point, y_liner, z_plane, 2);
        }
        Section::point_type p1(x_begin, y_liner);
        Section::point_type p2(x_point, y_liner);
        Section::state p2_state = z_section.judge_p2(p1, p2, j);
        if (p2_state == Section::state::ONBOUNDARY) {
          ps_(i, j, k) = node(x_point, y_liner, z_plane, 1);
        }
        else if (p2_state == Section::state::INNER) {
          ps_(i, j, k) = node(x_point, y_liner, z_plane, 0);
        }
        else if (p2_state == Section::state::OUTER) {
          ps_(i, j, k) = node(x_point, y_liner, z_plane, 2);
        }
        else {
          capture.log_fatal(2, "status shoult not be coincidence!");
        }
      }
    }
  }
  get_result_ = true;
}

std::vector<size_t> StlEntity::try_disperse(double grid_size) {
  if (grid_size <= 0) {
    capture.log_fatal(2, piece("grid_size invalid : ", grid_size));
  }
  //每次划分，需要获得一个新的边界盒，原先的边界盒紧贴stl实体，容易产生误差。
  InclusionBox box = _get_inclusion_box_from_grid_size_(grid_size);
  size_t x_count = static_cast<size_t>((box.max_x - box.least_x) / grid_size) + 1;
  size_t y_count = static_cast<size_t>((box.max_y - box.least_y) / grid_size) + 1;
  size_t z_count = static_cast<size_t>((box.max_z - box.least_z) / grid_size) + 1;
  return { x_count,y_count,z_count };
}

void StlEntity::out_to_tecplot(std::string filename) {
  std::ofstream out(filename.c_str());
  if (out.good() == false) {
    capture.log_fatal(2, piece("file open error : ", filename.c_str()));
  }
  if (get_result_ == false) {
    capture.log_fatal(2, piece("out before get!"));
  }
  ps_.out_to_tecplot(out);
  out.close();
}

double StlEntity::_get_real_y_liner_(double yl, size_t x_count, InclusionBox& box, double grid_size, Section& z_secion, size_t j) {
  bool error = true;
  double y_liner = yl;
  size_t error_times = 0;
  while (error == true && error_times < 10) {
    for (size_t i = 1; i < x_count; ++i) {
      error = false;
      double x_point = box.least_x + i * grid_size;
      double x_begin = box.least_x;
      Section::point_type p1(x_begin, y_liner);
      Section::point_type p2(x_point, y_liner);
      Section::state p2_state = z_secion.judge_p2(p1, p2, j);
      if (p2_state == Section::state::COINCIDENCE) {
        y_liner += 0.02;
        ++error_times;
        error = true;
        break;
      }
    }
    if (error == false) {
      break;
    }
  }
  if (error_times == 10) {
    capture.log_fatal(2, __LINE__, __FILE__, piece("get real y liner error!", y_liner));
  }
  return y_liner;
}
//输入一个三角面片索引和属于其的一条边的索引，返回和该三角形共该边的另一个三角形的顶点索引，
//该顶点不在该边上。
size_t StlEntity::_get_other_point_on_other_tria(size_t tria_index, size_t edge_index) {
  size_t tria_index1 = edge_set_.at(edge_index).tria_index_[0];
  size_t tria_index2 = edge_set_.at(edge_index).tria_index_[1];
  size_t tria_index_;
  if (tria_index1 == tria_index) {
    tria_index_ = tria_index2;
  }
  else if (tria_index2 == tria_index) {
    tria_index_ = tria_index1;
  }
  else {
    capture.log_fatal(2, __LINE__, __FILE__, piece("get other point on other tria error! : ", tria_index));
  }
  size_t p1 = tria_set_.at(tria_index_).point_index_[0];
  size_t p2 = tria_set_.at(tria_index_).point_index_[1];
  size_t p3 = tria_set_.at(tria_index_).point_index_[2];

  size_t ep1 = edge_set_.at(edge_index).point_one;
  size_t ep2 = edge_set_.at(edge_index).point_two;
  if (p1 != ep1 && p1 != ep2) {
    return p1;
  }
  else if (p2 != ep1 && p2 != ep2) {
    return p2;
  }
  else {
    return p3;
  }
}

//如果当前z_plane处于特殊位置，需要进行微调（例如有三角面刚好落在该平面上）
double StlEntity::_get_real_z_plane_(double zp) {
  bool error = true;
  int times = 0;
  while (error == true && times <= 10) {
    error = false;
    for (auto it = tria_set_.begin(); it != tria_set_.end(); ++it) {
      point& p1 = point_set_[it->point_index_[0]];
      point& p2 = point_set_[it->point_index_[1]];
      point& p3 = point_set_[it->point_index_[2]];
      if (value_equal(p1.z, zp) && value_equal(p2.z, zp) && value_equal(p3.z, zp)) {
        error = true;
        break;
      }
    }
    if (error == false)
      break;
    ++times;
    zp += 0.1;
  }
  if (times == 10) {
    capture.log_fatal(2, piece("z plane error! : ", zp));
  }
  return zp;
}

//将三角面片按照分布区间分别放入三个维度的桶里。
void StlEntity::_tri_classify_(double x_min, size_t x_count, double y_min, size_t y_count, double z_min, size_t z_count, double grid_size) {
  tri_bucket_x_.resize(x_count);
  tri_bucket_y_.resize(y_count);
  tri_bucket_z_.resize(z_count);
  size_t tri_index = 0;
  for (auto it = tria_set_.begin(); it != tria_set_.end(); ++it) {
    std::vector<point> tmpx = _order_by_x_(tri_index);
    std::vector<point> tmpy = _order_by_y_(tri_index);
    std::vector<point> tmpz = _order_by_z_(tri_index);

    for (size_t i = 0; i < x_count; ++i) {
      double x = x_min + i * grid_size;
      double x_next = x + grid_size;
      if (value_b(tmpx[2].x, x_next) || value_s(tmpx[0].x, x)) {
        ;
      }
      else {
        tri_bucket_x_[i].push_back(tri_index);
      }
    }

    for (size_t j = 0; j < y_count; ++j) {
      double y = y_min + j * grid_size;
      double y_next = y + grid_size;
      if (value_b(tmpy[2].y, y_next) || value_s(tmpy[0].y, y)) {
        ;
      }
      else {
        tri_bucket_y_[j].push_back(tri_index);
      }
    }

    for (size_t k = 0; k < z_count; ++k) {
      double z = z_min + k * grid_size;
      double z_next = z + grid_size;
      if (value_b(tmpz[2].z, z_next) || value_s(tmpz[0].z, z)) {
        ;
      }
      else {
        tri_bucket_z_[k].push_back(tri_index);
      }
    }
    ++tri_index;
  }
}

//result[0].x >= result[1].x >= result[2].x
std::vector<StlEntity::point> StlEntity::_order_by_x_(const point& p1, const point& p2, const point& p3) {
  std::vector<point> result;
  result.push_back(p1);
  result.push_back(p2);
  result.push_back(p3);
  if (result[0].x < result[1].x) {
    std::swap(result[0], result[1]);
  }
  if (result[1].x < result[2].x) {
    std::swap(result[1], result[2]);
  }
  if (result[0].x < result[1].x) {
    std::swap(result[0], result[1]);
  }
  return result;
}

std::vector<StlEntity::point> StlEntity::_order_by_x_(size_t tria_index) {
  point& p1 = point_set_.at(tria_set_.at(tria_index).point_index_[0]);
  point& p2 = point_set_.at(tria_set_.at(tria_index).point_index_[1]);
  point& p3 = point_set_.at(tria_set_.at(tria_index).point_index_[2]);
  return _order_by_x_(p1, p2, p3);
}

std::vector<StlEntity::point> StlEntity::_order_by_y_(const point& p1, const point& p2, const point& p3) {
  std::vector<point> result;
  result.push_back(p1);
  result.push_back(p2);
  result.push_back(p3);
  if (result[0].y < result[1].y) {
    std::swap(result[0], result[1]);
  }
  if (result[1].y < result[2].y) {
    std::swap(result[1], result[2]);
  }
  if (result[0].y < result[1].y) {
    std::swap(result[0], result[1]);
  }
  return result;
}

std::vector<StlEntity::point> StlEntity::_order_by_y_(size_t tria_index) {
  point& p1 = point_set_.at(tria_set_.at(tria_index).point_index_[0]);
  point& p2 = point_set_.at(tria_set_.at(tria_index).point_index_[1]);
  point& p3 = point_set_.at(tria_set_.at(tria_index).point_index_[2]);
  return _order_by_y_(p1, p2, p3);
}

std::vector<StlEntity::point> StlEntity::_order_by_z_(const point& p1, const point& p2, const point& p3) {
  std::vector<point> result;
  result.push_back(p1);
  result.push_back(p2);
  result.push_back(p3);
  if (result[0].z < result[1].z) {
    std::swap(result[0], result[1]);
  }
  if (result[1].z < result[2].z) {
    std::swap(result[1], result[2]);
  }
  if (result[0].z < result[1].z) {
    std::swap(result[0], result[1]);
  }
  return result;
}

std::vector<StlEntity::point> StlEntity::_order_by_z_(size_t tria_index) {
  point& p1 = point_set_.at(tria_set_.at(tria_index).point_index_[0]);
  point& p2 = point_set_.at(tria_set_.at(tria_index).point_index_[1]);
  point& p3 = point_set_.at(tria_set_.at(tria_index).point_index_[2]);
  return _order_by_z_(p1, p2, p3);
}

StlEntity::InclusionBox StlEntity::_get_inclusion_box_from_grid_size_(double grid_size) {
  InclusionBox result;
  double deviation_ = 4 * grid_size;
  result.least_x = box_.least_x - deviation_;
  result.least_y = box_.least_y - deviation_;
  result.least_z = box_.least_z - deviation_;
  result.max_x = box_.max_x + deviation_;
  result.max_y = box_.max_y + deviation_;
  result.max_z = box_.max_z + deviation_;
  return result;
}

void StlEntity::_create_tria_set_and_point_set_() {
  for (auto it = triangles_.begin(); it != triangles_.end(); ++it) {
    point tmp;
    tria triatmp;
    //填充法向量
    triatmp.normal[0] = it->normal[0];
    triatmp.normal[1] = it->normal[1];
    triatmp.normal[2] = it->normal[2];
    //将此时的三角面push到tria_set_中，此刻该三角面片只填充了法向量信息，没有更新点集和边集。
    tria_set_.push_back(triatmp);
    //更新该点的tria_index_,即triatmp的index。
    tmp.tria_index_.push_back(tria_set_.size() - 1);
    //分别将三个点插入point_set_
    tmp.x = it->first[0];
    tmp.y = it->first[1];
    tmp.z = it->first[2];

    point_set_.push_back(tmp);
    tmp.x = it->second[0];
    tmp.y = it->second[1];
    tmp.z = it->second[2];

    point_set_.push_back(tmp);
    tmp.x = it->third[0];
    tmp.y = it->third[1];
    tmp.z = it->third[2];

    point_set_.push_back(tmp);
  }
}

void StlEntity::_merge_point_set_() {
  try {
    //通过排序将相同节点在point_set_中相邻。
    std::sort(point_set_.begin(), point_set_.end());
  }
  catch (std::exception& e) {
    capture.log_fatal(1, piece(e.what()));
  }
  std::vector<point> new_set;
  std::vector<point>::iterator every_last;
  for (auto it = point_set_.begin(); it != point_set_.end(); ++it) {
    if (it == point_set_.begin()) {
      every_last = it;
    }
    else {
      if (*it == *every_last) {
        //此时每个point只所属于一个三角面片，故获得begin的index即可。
        size_t index = (*((it->tria_index_).begin()));
        //every_last继承相同节点的index。
        every_last->tria_index_.push_back(index);
      }
      else {//如果it节点是新的节点，则意味着上一个节点合并已经完成，将这个节点push进new_set，然后将此个节点作为新的every_last节点。
        new_set.push_back(*every_last);
        every_last = it;
      }
      //处理结束情况。
      if (it == point_set_.end() - 1) { //如果it节点是最后一个节点，将其push进new_set即可。
        new_set.push_back(*every_last);
      }
    }
  }
  new_set.swap(point_set_);
}

void StlEntity::_set_point_index_of_tria_set_() {
  size_t point_index = 0;
  for (auto it = point_set_.begin(); it != point_set_.end(); ++it) {
    for (auto it2 = it->tria_index_.begin(); it2 != it->tria_index_.end(); ++it2) {
      tria_set_.at(*it2).point_index_.push_back(point_index);
    }
    ++point_index;
  }
}

void StlEntity::_create_edge_set_() {
  size_t tria_index = 0;
  for (auto it = tria_set_.begin(); it != tria_set_.end(); ++it) {
    if (it->point_index_.size() != 3) {
      capture.log_fatal(2, piece("tria ", tria_index, "'s point_index_ size error : ", it->point_index_.size()));
    }
    size_t point_1 = it->point_index_[0];
    size_t point_2 = it->point_index_[1];
    size_t point_3 = it->point_index_[2];
    edge tmp;

    tmp.tria_index_.push_back(tria_index);
    tmp.point_one = the_smaller(point_1, point_2);
    tmp.point_two = the_bigger(point_1, point_2);
    edge_set_.push_back(tmp);


    tmp.point_one = the_smaller(point_1, point_3);
    tmp.point_two = the_bigger(point_1, point_3);
    edge_set_.push_back(tmp);


    tmp.point_one = the_smaller(point_2, point_3);
    tmp.point_two = the_bigger(point_2, point_3);
    edge_set_.push_back(tmp);

    ++tria_index;
  }
}

void StlEntity::_merge_edge_set_() {
  try {
    //通过排序将相同节点在point_set_中相邻。
    std::sort(edge_set_.begin(), edge_set_.end());
  }
  catch (std::exception& e) {
    capture.log_fatal(1, piece(e.what()));
  }
  std::vector<edge> new_edge_set;
  std::vector<edge>::iterator edge_last;
  for (auto it = edge_set_.begin(); it != edge_set_.end(); ++it) {
    if (it == edge_set_.begin()) {
      edge_last = it;
    }
    else {
      if (it->point_one == edge_last->point_one && it->point_two == edge_last->point_two) {
        size_t index = (*((it->tria_index_).begin()));//此时每条边也只属于一个三角面片。
        edge_last->tria_index_.push_back(index);
      }
      else {
        new_edge_set.push_back(*edge_last);
        edge_last = it;
      }
    }
    if (it == edge_set_.end() - 1) {
      new_edge_set.push_back(*edge_last);
    }
  }
  new_edge_set.swap(edge_set_);
}

void StlEntity::_set_edge_index_for_tria_and_point_set_() {
  size_t edge_index = 0;
  for (auto it = edge_set_.begin(); it != edge_set_.end(); ++it) {
    for (auto it2 = it->tria_index_.begin(); it2 != it->tria_index_.end(); ++it2) {
      tria_set_.at(*it2).edge_index_.push_back(edge_index);
    }
    point_set_[it->point_one].edge_index_.push_back(edge_index);
    point_set_[it->point_two].edge_index_.push_back(edge_index);
    ++edge_index;
  }
}