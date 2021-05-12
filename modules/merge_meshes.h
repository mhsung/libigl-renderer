// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_MERGE_MESHES_H
#define IGL_MERGE_MESHES_H

#include <igl/igl_inline.h>

#include <assert.h>
#include <Eigen/Core>

namespace igl
{
  template <
    typename DerivedV,
    typename DerivedF>
  IGL_INLINE void merge_meshes(
    const DerivedV& V1, const DerivedF& F1,
    const DerivedV& V2, const DerivedF& F2,
    DerivedV& merged_V, DerivedF& merged_F);

  template <
    typename MatV,
    typename MatF>
  IGL_INLINE void add_mesh(
    MatV& V,
    MatF& F,
    const MatV& other_V,
    const MatF& other_F);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "merge_meshes.cpp"
//#endif


template <
    typename DerivedV,
    typename DerivedF>
IGL_INLINE void igl::merge_meshes(
    const DerivedV& V1, const DerivedF& F1,
    const DerivedV& V2, const DerivedF& F2,
    DerivedV& merged_V, DerivedF& merged_F)
{
  if (V1.rows() == 0) {
    assert (F1.rows() == 0);
    merged_V = V2;
    merged_F = F2;
    return;
  }
  else if (V2.rows() == 0) {
    assert (F2.rows() == 0);
    merged_V = V1;
    merged_F = F1;
    return;
  }

  assert(V1.cols() == V2.cols());
  merged_V.resize(V1.rows() + V2.rows(), V1.cols());
  merged_V << V1, V2;

  const DerivedF& shifted_F2 = F2.array() + V1.rows();

  if (F1.rows() == 0) {
    merged_F = shifted_F2;
    return;
  }
  else if (F2.rows() == 0) {
    merged_F = F1;
    return;
  }

  assert(F1.cols() == F2.cols());
  merged_F.resize(F1.rows() + F2.rows(), F1.cols());
  merged_F << F1, shifted_F2;
}

template <
  typename MatV,
  typename MatF>
IGL_INLINE void igl::add_mesh(
  MatV& V,
  MatF& F,
  const MatV& other_V,
  const MatF& other_F)
{
  const MatV V_copy = V;
  const MatF F_copy = F;
  igl::merge_meshes(V_copy, F_copy, other_V, other_F, V, F);
}

#endif
