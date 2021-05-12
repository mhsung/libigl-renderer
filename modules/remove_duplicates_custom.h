// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_REMOVE_DUPLICATES_CUSTOM_H
#define IGL_REMOVE_DUPLICATES_CUSTOM_H

#include <igl/igl_inline.h>
#include <igl/sortrows.h>
#include <utils/kd_tree.h>

#include <Eigen/Core>

namespace igl
{
  // NOTE:
  // See 'remove_duplicate_vertices.h' and 'resolve_duplicated_faces.h' as well.
  // 'remove_duplicate_vertices_custom()' function can be replaced with
  // 'remove_duplicate_vertices()'.
  // Didn't check whether both work in the same way.
  // 'remove_duplicate_faces_custom()' works differently with
  // 'resolve_duplicated_faces()'.

  template <
    typename DerivedV,
    typename DerivedF>
  IGL_INLINE void remove_duplicate_vertices_custom(
    const DerivedV& V, const DerivedF& F,
    DerivedV& newV, DerivedF& newF,
    const float squared_eps = 1.0E-8f,
    const int max_K = 16);

  template <
    typename MatV,
    typename MatF>
  IGL_INLINE void remove_duplicate_vertices_custom(
    MatV& V,
    MatF& F,
    const float squared_eps = 1.0E-8f,
    const int max_K = 16);

  template <typename DerivedF>
  IGL_INLINE void remove_duplicate_faces_custom(
    const DerivedF& F, DerivedF& newF);

  template <typename MatF>
  IGL_INLINE void remove_duplicate_faces_custom(
    MatF& F);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "remove_duplicates_custom.cpp"
//#endif


template <
    typename DerivedV,
    typename DerivedF>
IGL_INLINE void igl::remove_duplicate_vertices_custom(
    const DerivedV& V, const DerivedF& F,
    DerivedV& newV, DerivedF& newF,
    const float squared_eps,
    const int max_K)
{
  // Assume triangular mesh.
  assert(F.cols() == 3);

  // Find the second closest points in the same vertex set.
  KdTree::KdTree<DerivedV> P_tree(V);
  MatrixXi closest_idxs;
  MatrixXd closest_sqr_dists;
  KdTree::find_k_closest_points(
      P_tree, V, &closest_idxs, &closest_sqr_dists, max_K);

  // Merge close vertices.
  const int nV = V.rows();
  std::vector<int> old_to_new_vids(nV, -1);
  std::vector<int> new_to_old_vids;
  new_to_old_vids.reserve(nV);

  for (int vid = 0; vid < nV; ++vid) {
    bool close_to_prev_v = false;
    for (int i = 0; i < closest_idxs.cols(); ++i) {
      const int r_vid = closest_idxs(vid, i);
      const double sqr_dist = closest_sqr_dists(vid, i);
      if (sqr_dist < squared_eps && r_vid < vid) {
        old_to_new_vids[vid] = old_to_new_vids[r_vid];
        close_to_prev_v = true;
        break;
      }
    }
    if (!close_to_prev_v) {
      old_to_new_vids[vid] = new_to_old_vids.size();
      new_to_old_vids.push_back(vid);
    }
  }

  const int new_nV = new_to_old_vids.size();
  if (new_nV == nV) {
    newV = V;
    newF = F;
    return;
  }

  newV.resize(new_nV, V.cols());
  for (int vid = 0; vid < new_nV; ++vid) {
    newV.row(vid) = V.row(new_to_old_vids[vid]);
  }

  newF.resize(F.rows(), F.cols());
  for (int fid = 0; fid < F.rows(); ++fid) {
    newF.row(fid) << old_to_new_vids[ F.row(fid)[0] ],
        old_to_new_vids[ F.row(fid)[1] ],
        old_to_new_vids[ F.row(fid)[2] ];
  }
}

template <
    typename MatV,
    typename MatF>
IGL_INLINE void igl::remove_duplicate_vertices_custom(
    MatV& V,
    MatF& F,
    const float squared_eps,
    const int max_K)
{
  const MatV V_copy = V;
  const MatF F_copy = F;
  remove_duplicate_vertices_custom(V_copy,F_copy,V,F,squared_eps,max_K);
}

template <typename DerivedF>
IGL_INLINE void igl::remove_duplicate_faces_custom(
    const DerivedF& F, DerivedF& newF)
{
  // Assume triangular mesh.
  assert(F.cols() == 3);

  // Sort vertex indices of each face. The order of vertices is ignored.
  DerivedF F_col_sorted;
  Eigen::MatrixXi col_I;
  igl::sort(F, 2, true, F_col_sorted, col_I);

  DerivedF F_row_sorted;
  Eigen::MatrixXi row_I;
  igl::sortrows(F_col_sorted, true, F_row_sorted, row_I);

  // Merge identical faces.
  const int nF = F.rows();
  std::vector<int> new_to_old_fids;
  new_to_old_fids.reserve(nF);

  int prev_i = -1;
  for (int i = 0; i < nF; ++i) {
    const int fid = row_I(i);

    bool identical_with_prev_f = false;
    if (prev_i >= 0) {
      const auto& f = F_row_sorted.row(i);
      const auto& prev_f = F_row_sorted.row(prev_i);
      if ((f.array() == prev_f.array()).all()) {
        identical_with_prev_f = true;
      }
    }
    if (!identical_with_prev_f) {
      new_to_old_fids.push_back(fid);
      prev_i = i;
    }
  }

  const int new_nF = new_to_old_fids.size();
  if (new_nF == nF) {
    newF = F;
    return;
  }

  newF.resize(new_nF, F.cols());
  for (int fid = 0; fid < new_nF; ++fid) {
    newF.row(fid) = F.row(new_to_old_fids[fid]);
  }
};

template <typename MatF>
IGL_INLINE void igl::remove_duplicate_faces_custom(
    MatF& F)
{
  const MatF F_copy = F;
  remove_duplicate_faces_custom(F_copy,F);
}

#endif
