// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_SYMMETRIC_ELEMENTS_H
#define IGL_SYMMETRIC_ELEMENTS_H

#include <igl/igl_inline.h>
#include <modules/edge_topology_non_manifold.h>

#include <Eigen/Core>
#include <utils/kd_tree.h>

namespace igl
{
  // Input:
  // V: N x 3 vertices.
  // ST: 3 x 3 symmetry transformation matrix.
  // SV^T = ST * V^T => SV = V * ST^T
  // Output:
  // SVI: Symmetric vertex indices. -1 if it does not exist.
  template <
      typename DerivedV,
      typename DerivedT>
  IGL_INLINE void symmetric_vertices(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedT>& ST,
    Eigen::VectorXi& SVI,
    const float squared_eps = 1.0E-6f);

  template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedT>
  IGL_INLINE void symmetric_faces(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F,
    const Eigen::PlainObjectBase<DerivedT>& ST,
    Eigen::VectorXi& SFI,
    const float squared_eps = 1.0E-6f);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "symmetric_elements.cpp"
//#endif

template <
    typename DerivedV,
    typename DerivedT>
IGL_INLINE void igl::symmetric_vertices(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedT>& ST,
    Eigen::VectorXi& SVI,
    const float squared_eps)
{
  const KdTree::KdTree<DerivedV> V_tree(V);
  const Eigen::PlainObjectBase<DerivedV> SV = V * ST.transpose();
  Eigen::VectorXd sqr_dists;
  KdTree::find_k_closest_points(V_tree, SV, &SVI, &sqr_dists);

  int num_symm_verts = 0;
  for (int vid = 0; vid < V.rows(); ++vid) {
    if (sqr_dists[vid] > squared_eps || SVI[vid] == vid) SVI[vid] = -1;
    else ++num_symm_verts;
  }
  LOG(INFO) << "# symmetric vertices: " << num_symm_verts;
}

template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedT>
IGL_INLINE void igl::symmetric_faces(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F,
    const Eigen::PlainObjectBase<DerivedT>& ST,
    Eigen::VectorXi& SFI,
    const float squared_eps)
{
  // Assume triangular mesh.
  assert(F.cols() == 3);
  typedef typename DerivedF::Scalar ScalarF;

  Eigen::VectorXi SVI;
  symmetric_vertices(V, ST, SVI, squared_eps);

  // Sort vertex indices of each face. The order of vertices is ignored.
  Eigen::PlainObjectBase<DerivedF> F_col_sorted;
  Eigen::MatrixXi col_I;
  igl::sort(F, 2, true, F_col_sorted, col_I);

  // Construct k-d tree using face vertex indices.
  const KdTree::KdTree<DerivedF> F_tree(F_col_sorted);

  const int nF = F.rows();
  SFI = -1 * Eigen::VectorXi::Ones(nF);

  int num_symm_faces = 0;
  for (int fid = 0; fid < nF; ++fid) {
    // Find symmetric faces based on symmetric vertices.
    Matrix<ScalarF, 1, 3> symm_f(
        SVI[F(fid, 0)], SVI[F(fid, 1)], SVI[F(fid, 2)]);
    if ((symm_f.array() < 0).any()) continue;
    Matrix<ScalarF, 1, 3> sorted_symm_f;
    Eigen::MatrixXi symm_f_idxs;
    igl::sort(symm_f, 2, true, sorted_symm_f, symm_f_idxs);

    Eigen::VectorXi idxs;
    Eigen::VectorXd sqr_dists;
    KdTree::find_k_closest_points(F_tree, sorted_symm_f, &idxs, &sqr_dists);
    if (sqr_dists[0] == 0 && idxs[0] != fid) {
     SFI[fid] = idxs[0];
      ++num_symm_faces;
    }
  }
  LOG(INFO) << "# symmetric faces: " << num_symm_faces;
};

#endif
