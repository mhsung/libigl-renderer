// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "LibiglMesh.h"
#include <Eigen/Geometry>
#include <utils/utils.h>


// Mesh processing.
DEFINE_bool(centerize, false, "");
DEFINE_bool(flip_x, false, "");
DEFINE_bool(flip_y, false, "");
DEFINE_bool(flip_z, false, "");
DEFINE_string(translation, "", "(tx, ty, tz)");
DEFINE_string(rotation, "", "(rx, ry, rz)");
DEFINE_string(transformation, "", "(rx, ry, rz, tx, ty, tz)");
DEFINE_string(inverse_transformation, "", "(rx, ry, rz, tx, ty, tz)");
DEFINE_bool(normalize_height, false, "");

// Point set processing.
DEFINE_bool(sample_points, false, "");
DEFINE_bool(with_normals, false, "");
DEFINE_int32(num_sample_points, 1024, "");
DEFINE_bool(normalize_point_set, false, "");
DEFINE_bool(centerize_point_set, false, "");
DEFINE_string(out_point_set_center, "", "");
DEFINE_bool(pca_align_point_set, false, "");
DEFINE_string(out_pca_transformation, "", "");


void LibiglMesh::mesh_processing() {
  bool mesh_modified = false;

  // Centerize.
  if (FLAGS_centerize) {
    V_ = V_.rowwise() - center_.transpose();
    mesh_modified = true;
  }

  // Mesh flipping.
  if (FLAGS_flip_x) { V_.col(0) = -V_.col(0); mesh_modified = true; }
  if (FLAGS_flip_y) { V_.col(1) = -V_.col(1); mesh_modified = true; }
  if (FLAGS_flip_z) { V_.col(2) = -V_.col(2); mesh_modified = true; }

  if (FLAGS_translation != "") {
    std::vector<std::string> strs = Utils::split_string(FLAGS_translation);
    CHECK_EQ(strs.size(), 3);
    Vector3d t;
    for (int i = 0; i < 3; ++i) t[i] = std::stod(strs[i]);
    translate_mesh(V_, t);
    mesh_modified = true;
  }

  if (FLAGS_rotation != "") {
    std::vector<std::string> strs = Utils::split_string(FLAGS_rotation);
    CHECK_EQ(strs.size(), 3);
    Matrix3d R = Matrix3d::Identity();
    for (int i = 0; i < 3; ++i) {
      const double angle = std::stod(strs[i]) / 180.0 * M_PI;
      const AngleAxisd axis_R(angle, Vector3d::Unit(i));
      R = axis_R.toRotationMatrix() * R;
    }
    const Eigen::Matrix<double, Dynamic, 3>& V_temp = V_;
    V_ = (R * V_temp.transpose()).transpose();
    mesh_modified = true;
  }

  if (FLAGS_transformation != "") {
    std::vector<std::string> strs = Utils::split_string(FLAGS_transformation);
    CHECK_EQ(strs.size(), 6);
    Vector3d r, t;
    for (int i = 0; i < 3; ++i) r[i] = std::stod(strs[0 + i]);
    for (int i = 0; i < 3; ++i) t[i] = std::stod(strs[3 + i]);
    transform_mesh(V_, r, t);
    mesh_modified = true;
  }

  if (FLAGS_inverse_transformation != "") {
    std::vector<std::string> strs = Utils::split_string(
        FLAGS_inverse_transformation);
    CHECK_EQ(strs.size(), 6);
    Vector3d r, t;
    for (int i = 0; i < 3; ++i) r[i] = std::stod(strs[0 + i]);
    for (int i = 0; i < 3; ++i) t[i] = std::stod(strs[3 + i]);
    inverse_transform_mesh(V_, r, t);
    mesh_modified = true;
  }

  if (FLAGS_normalize_height) {
    const double height = V_.col(1).maxCoeff() - V_.col(1).minCoeff();
    CHECK_GT(height, 1.0E-6);
    V_ /= height;
  }

  if (mesh_modified) {
    update_bounding_box();
    renderer_->set_mesh(V_, F_);
    renderer_->set_scene_pos(center_.cast<float>(), (float)radius_);
  }
}

void LibiglMesh::point_set_processing() {
  if (FLAGS_sample_points) {
    sample_points_on_mesh(FLAGS_num_sample_points, FLAGS_with_normals);
  }

  if (FLAGS_normalize_point_set) {
    normalize_points();
  }

  if (FLAGS_centerize_point_set ||
      FLAGS_out_point_set_center != "") {
    compute_point_set_center_and_area(
        FLAGS_out_point_set_center, FLAGS_centerize_point_set);
  }

  if (FLAGS_pca_align_point_set) {
    pca_align_points(FLAGS_out_pca_transformation);
  }
}

void LibiglMesh::processing() {
  mesh_processing();
  point_set_processing();
}
