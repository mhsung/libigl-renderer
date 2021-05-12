// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_PCA_H
#define IGL_PCA_H
#include <igl/igl_inline.h>

#include <Eigen/Dense>

namespace igl
{
  // Run PCA.
  // P: n x k points.
  // T: Affine transformation matrix.
  // Aligned_P_transpose = T * P_transpose.
  //
  template <typename Scalar, int RowP, int ColumnP>
  IGL_INLINE void PCA(
    const Eigen::Matrix<Scalar, RowP, ColumnP>& P,
    Eigen::Transform<Scalar, 3, Eigen::Affine>& T);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "PCA.cpp"
//#endif


template <typename Scalar, int RowP, int ColumnP>
IGL_INLINE void igl::PCA(
    const Eigen::Matrix<Scalar, RowP, ColumnP>& P,
    Eigen::Transform<Scalar, 3, Eigen::Affine>& T)
{
  assert(P.cols() == 3);
  using namespace Eigen;

  const Matrix<Scalar, ColumnP, RowP> P_t = P.transpose();
  const Matrix<Scalar, 3, 1>  t_t(P_t.rowwise().mean());

  const Matrix<Scalar, ColumnP, RowP> P_t_centered = P_t.colwise() - t_t;
  JacobiSVD<Matrix<Scalar, ColumnP, RowP>> svd(P_t_centered, ComputeFullU);

  // NOTE:
  // Singular values are always sorted in decreasing order.
  // https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html
  Matrix<Scalar, 3, 3> R_t = svd.matrixU();
  if ((R_t.col(0).cross(R_t.col(1))).dot(R_t.col(2)) < 0)
    R_t.col(2) = -R_t.col(2);

  T.setIdentity();
  T.pretranslate(-t_t);
  T.prerotate(R_t.transpose());

  // NOTE:
  // Assume that the point set is flipped along an axis if more than half of
  // points have negative coordinates.
  // Since most components are given as aligned with PCA axes,
  // we slightly increase the threshold of flipping (0.5 + 0.01).
  const Eigen::Matrix<double, Dynamic, 3>& P_temp = P;
  const Eigen::Matrix<double, Dynamic, 3> P_aligned =
      (T * P_temp.transpose()).transpose();

  const int num_points = P.rows();
  Vector3i count_minus_coords = Vector3i::Zero();
  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (P_aligned(i, j) < 0) ++count_minus_coords[j];
    }
  }

  const bool x_flipped = count_minus_coords[0] > (0.5 + 0.01) * num_points;
  const bool y_flipped = count_minus_coords[1] > (0.5 + 0.01) * num_points;

  Matrix3d R_f = Matrix3d::Identity();
  if (!x_flipped && y_flipped) {
    // Rotate along x-axis: (x, y, z) -> (x, -y, -z)
    R_f = AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();
  } else if (x_flipped && !y_flipped) {
    // Rotate along y-axis: (x, y, z) -> (-x, y, -z)
    R_f = AngleAxisd(M_PI, Vector3d::UnitY()).toRotationMatrix();
  } else if(x_flipped && y_flipped) {
    // Rotate along z-axis: (x, y, z) -> (-x, -y, z)
    R_f = AngleAxisd(M_PI, Vector3d::UnitZ()).toRotationMatrix();
  }

  T.prerotate(R_f);
};

#endif

