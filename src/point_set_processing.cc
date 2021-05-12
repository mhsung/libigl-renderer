// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "LibiglMesh.h"

#include <Eigen/Geometry>
#include <igl/doublearea.h>
#include <igl/per_face_normals.h>
#include <igl/random_points_on_mesh.h>
#include <modules/PCA.h>
#include <modules/remove_duplicates_custom.h>
#include <utils/utils.h>


void LibiglMesh::sample_points_on_mesh(
    const int _num_points, const bool _with_normals) {
  // NOTE: 03-24-2017
  // Remove duplicated faces in mesh before sampling points.
  MatrixXi newF;
  igl::remove_duplicate_faces_custom(F_, newF);

  SparseMatrix<double> B;
  // NOTE: 02-25-2018
  // Assign face indices to points labels.
  igl::random_points_on_mesh(_num_points, V_, newF, B, PL_);
  const VectorXi& FI = PL_;
  P_ = B * V_;

  if (_with_normals) {
    igl::per_face_normals(V_, F_, FN_);
    PN_ = Utils::slice_rows(FN_, FI);
  }
}

void LibiglMesh::normalize_points() {
  const int num_samples = P_.rows();
  CHECK_GT(num_samples, 0);

  // Compute center and bounding box diagonal.
  const auto bb_min = P_.colwise().minCoeff();
  const auto bb_max = P_.colwise().maxCoeff();

  const RowVector3d bb_center = 0.5 * (bb_min + bb_max);
  P_ = P_.rowwise() - bb_center;

  const auto radius = P_.rowwise().norm().maxCoeff();
  P_ = P_ / radius;
}

void LibiglMesh::compute_point_set_center_and_area(
    const std::string& _out_file, bool _centerize) {
  const int num_samples = P_.rows();
  CHECK_GT(num_samples, 0);

  // Compute center and bounding box diagonal.
  const auto bb_min = P_.colwise().minCoeff();
  const auto bb_max = P_.colwise().maxCoeff();

  // NOTE: 03-24-2017
  // Use bounding box center as center instead of mean of point positions.
  //const RowVector3d center = P_.colwise().mean();
  const RowVector3d bb_center = 0.5 * (bb_min + bb_max);


  // NOTE: 03-24-2017
  // Use sum of face areas as size instead of bounding box diagonal.
  //const double bbox_diagonal = (bb_max - bb_min).norm();
  MatrixXi newF;
  igl::remove_duplicate_faces_custom(F_, newF);
  VectorXd FA;
  igl::doublearea(V_, newF, FA);
  const double sum_face_areas = 0.5 * FA.sum();

  if (_centerize) {
    // Centerize point set.
    P_ = P_.rowwise() - bb_center;
  }

  if (_out_file != "") {
    RowVector4d center_and_area;
    //center_and_area << center, bbox_diagonal;
    center_and_area << bb_center, sum_face_areas;
    Utils::write_eigen_matrix_to_file(_out_file, center_and_area);
  }
}

void LibiglMesh::pca_align_points(const std::string& _out_file) {
  const int num_samples = P_.rows();
  CHECK_GT(num_samples, 0);

  // Compute PCA transformation matrix.
  Affine3d T = Affine3d::Identity();
  igl::PCA(P_, T);

  // PCA-align point set.
  const Eigen::Matrix<double, Dynamic, 3>& P_temp = P_;
  P_ = (T * P_temp.transpose()).transpose();

  // NOTE:
  // P.transpose == R * P_aligned.transpose() + t
  Affine3d T_inv = T.inverse();
  const Matrix3d R = T_inv.rotation();
  const Vector3d t = T_inv.translation();

  const AngleAxisd rotation(R);
  double angle = rotation.angle();
  // Make angle to be in [-pi, +pi) range.
  while (angle < -M_PI) angle += (2.0 * M_PI);
  while (angle >= M_PI) angle -= (2.0 * M_PI);
  Vector3d axis = rotation.axis().normalized();

  const Vector3d r = angle * axis;

  // NOTE: 03-24-2017
  // Add sum of face areas.
  MatrixXi newF;
  igl::remove_duplicate_faces_custom(F_, newF);
  VectorXd FA;
  igl::doublearea(V_, newF, FA);
  const double a = 0.5 * FA.sum();

  // Scale along the first PCA axis.
  const double s = P_.col(0).maxCoeff() - P_.col(0).minCoeff();

  VectorXd transformation(8);
  transformation << r, t, a, s;
  if (_out_file != "") {
    Utils::write_eigen_matrix_to_file(_out_file, transformation.transpose());
  }
}
