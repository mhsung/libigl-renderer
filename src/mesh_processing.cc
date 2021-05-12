// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "LibiglMesh.h"

#include <Eigen/Geometry>
#include <utils/utils.h>


void LibiglMesh::normalize_mesh(MatrixXd& _V) {
  const auto bb_min = _V.colwise().minCoeff();
  const auto bb_max = _V.colwise().maxCoeff();
  const auto center = 0.5 * (bb_max + bb_min);
  const double bbox_diagonal = (bb_max - bb_min).norm();
  CHECK_GT(bbox_diagonal, 1.0E-6);

  // Move center to (0,0,0).
  _V = _V.rowwise() - center;

  // Scale to bounding box diagonal 1.
  _V /= bbox_diagonal;
}

void LibiglMesh::translate_mesh(MatrixXd& _V, const Vector3d& _t) {
  _V = _V.rowwise() + _t.transpose();
}

void LibiglMesh::transform_mesh(
    MatrixXd& _V, const Vector3d& _r, const Vector3d& _t) {
  const double kZeroTol = 1.0e-6;

  double angle = _r.norm();
  Vector3d axis;
  if (std::abs(angle) < kZeroTol) {
    angle = 0.0;
    axis = Vector3d::UnitX();
  } else {
    axis = _r.normalized();
  }
  AngleAxisd rotation(angle, axis);
  const Matrix3d R = rotation.toRotationMatrix();

  Affine3d T = Affine3d::Identity();
  T.prerotate(R);
  T.pretranslate(_t);

  const Eigen::Matrix<double, Dynamic, 3>& V_temp = _V;
  _V = (T * V_temp.transpose()).transpose();
}

void LibiglMesh::inverse_transform_mesh(
    MatrixXd& _V, const Vector3d& _r, const Vector3d& _t) {
  const double kZeroTol = 1.0e-6;

  double angle = _r.norm();
  Vector3d axis;
  if (std::abs(angle) < kZeroTol) {
    angle = 0.0;
    axis = Vector3d::UnitX();
  } else {
    axis = _r.normalized();
  }
  AngleAxisd rotation(angle, axis);
  const Matrix3d R = rotation.toRotationMatrix();

  Affine3d T_inv = Affine3d::Identity();
  T_inv.prerotate(R);
  T_inv.pretranslate(_t);
  const Affine3d T = T_inv.inverse();

  const Eigen::Matrix<double, Dynamic, 3>& V_temp = _V;
  _V = (T * V_temp.transpose()).transpose();
}

/*
void LibiglMesh::transform_mesh(const std::string& _filename) {
  Matrix4d mat;
  if (!Utils::read_eigen_matrix_from_file(_filename, &mat)) {
    return;
  }

  Affine3d T(mat);
  Matrix<double, Dynamic, 3> V_copy = V_;
  V_ = (T * V_copy.transpose()).transpose();

  update_bounding_box();
  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set";
  } else {
    renderer_->set_mesh(V_, F_);
    renderer_->set_scene_pos(center_.cast<float>(), (float)radius_);
  }
}
*/

