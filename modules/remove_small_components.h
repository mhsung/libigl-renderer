// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_REMOVE_SMALL_COMPONENTS_H
#define IGL_REMOVE_SMALL_COMPONENTS_H

#include <igl/igl_inline.h>
#include <igl/facet_components.h>
#include <igl/remove_unreferenced.h>
#include <igl/slice.h>
#include <igl/unique.h>
#include <utils/utils.h>

#include <map>
#include <vector>
#include <Eigen/Core>

namespace igl
{
  template <
    typename DerivedV,
    typename DerivedF>
  IGL_INLINE void remove_small_components(
    const DerivedV& V, const DerivedF& F,
    DerivedV& newV, DerivedF& newF,
    const float min_bbox_diag_ratio = 0.02f);

  template <
    typename MatV,
    typename MatF>
  IGL_INLINE void remove_small_components(
    MatV& V,
    MatF& F,
    const float min_bbox_diag_ratio = 0.02f);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "remove_small_components.cpp"
//#endif


template <
    typename DerivedV,
    typename DerivedF>
IGL_INLINE void igl::remove_small_components(
    const DerivedV& V, const DerivedF& F,
    DerivedV& newV, DerivedF& newF,
    const float min_bbox_diag_ratio)
{
  using namespace Eigen;
  typedef typename DerivedV::Scalar ScalarV;
  typedef typename DerivedF::Scalar ScalarF;

  // Find components.
  VectorXi C;
  igl::facet_components(F, C);

  VectorXi C_set;
  igl::unique(C, C_set);
  std::map<int, int> C_to_idx;
  const int num_C_set = C_set.size();
  for (int cid = 0; cid < num_C_set; ++cid) {
    C_to_idx[ C_set[cid] ] = cid;
  }

  // Compute component bbox diagonals.
  Matrix<ScalarV, Dynamic, 1> CBD(num_C_set);
  for (int cid = 0; cid < num_C_set; ++cid) {
    // FIXME(mhsung): Consider not to use utils functions.
    const Matrix<ScalarF, Dynamic, 1> C_fids = Utils::find(C, C_set[cid]);
    DerivedF CF;
    igl::slice(F, C_fids, 1, CF);
    const Map<Matrix<ScalarF, Dynamic, 1> > CF_vec(CF.data(), CF.size());
    Matrix<ScalarF, Dynamic, 1> C_vids;
    igl::unique(CF_vec, C_vids);
    DerivedV CV;
    igl::slice(V, C_vids, 1, CV);
    const auto C_bb_min = CV.colwise().minCoeff();
    const auto C_bb_max = CV.colwise().maxCoeff();
    CBD[cid] = (C_bb_max - C_bb_min).norm();
  }

  // Remove faces in small components.
  const ScalarV min_bbox_diag = min_bbox_diag_ratio * CBD.maxCoeff();
  std::vector<bool> is_component_small(num_C_set, false);
  for (int cid = 0; cid < num_C_set; ++cid) {
    if (CBD[cid] < min_bbox_diag) is_component_small[cid] = true;
  }

  const int nF = F.rows();
  std::vector<int> new_to_old_fids;
  new_to_old_fids.reserve(nF);
  for (int fid = 0; fid < nF; ++fid) {
    const int cid = C_to_idx[ C[fid] ];
    if (!is_component_small[cid]) new_to_old_fids.push_back(fid);
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

  const DerivedF newF_copy = newF;
  VectorXi IX;
  igl::remove_unreferenced(V, newF_copy, newV, newF, IX);
}

template <
    typename MatV,
    typename MatF>
IGL_INLINE void igl::remove_small_components(
    MatV& V,
    MatF& F,
    const float min_bbox_diag_ratio)
{
  const MatV V_copy = V;
  const MatF F_copy = F;
  remove_small_components(V_copy,F_copy,V,F,min_bbox_diag_ratio);
}

#endif
