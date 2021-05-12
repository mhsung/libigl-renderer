// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_EDGE_LENGTHS_SIMPLE_H
#define IGL_EDGE_LENGTHS_SIMPLE_H
#include <igl/igl_inline.h>

#include <Eigen/Dense>

namespace igl
{
  // Constructs a list of lengths of edges opposite each index in a face
  // (triangle/tet) list
  //
  template <typename DerivedV, typename DerivedE, typename DerivedL>
  IGL_INLINE void edge_lengths_simple(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedE>& EV,
    Eigen::PlainObjectBase<DerivedL>& EL);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "edge_lengths_simple.cpp"
//#endif


template <typename DerivedV, typename DerivedE, typename DerivedL>
IGL_INLINE void igl::edge_lengths_simple(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedE>& EV,
    Eigen::PlainObjectBase<DerivedL>& EL)
{
  const int n_edges = EV.rows();
  EL.resize(n_edges, 1);

  for (int i = 0; i < n_edges; ++i) {
    const auto& v1 = V.row(EV(i, 0));
    const auto& v2 = V.row(EV(i, 1));
    EL(i) = (v1 - v2).norm();
  }
};

#endif

