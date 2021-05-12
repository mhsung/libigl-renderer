// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_UPSAMPLE_NON_MANIFOLD_H
#define IGL_UPSAMPLE_NON_MANIFOLD_H

#include <igl/igl_inline.h>
#include <modules/edge_lengths_simple.h>
#include <modules/edge_topology_non_manifold.h>

#include <list>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace igl
{
  // Subdivide without moving vertices: Given the triangle mesh [V, F],
  // where n_verts = V.rows(), computes newV and a sparse matrix S s.t.
  // [newV, newF] is the subdivided mesh where newV = S*V.
  //
  template <
    typename DerivedV,
    typename DerivedF>
  IGL_INLINE void upsample_non_manifold(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F,
    Eigen::PlainObjectBase<DerivedV>& newV,
    Eigen::PlainObjectBase<DerivedF>& newF,
    const float min_edge_length = 0.0f);

  // Virtually in place wrapper
  template <
    typename MatV,
    typename MatF>
  IGL_INLINE void upsample_non_manifold(
    MatV& V,
    MatF& F,
    const float min_edge_length = 0.0f);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "upsample_non_manifold.cpp"
//#endif


template <
    typename DerivedV,
    typename DerivedF>
IGL_INLINE void igl::upsample_non_manifold(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F,
    Eigen::PlainObjectBase<DerivedV>& newV,
    Eigen::PlainObjectBase<DerivedF>& newF,
    const float min_edge_length)
{
  // Assume triangular mesh.
  assert(F.cols() == 3);
  using namespace std;
  using namespace Eigen;
  typedef typename DerivedV::Scalar ScalarV;
  typedef typename DerivedF::Scalar ScalarF;

  MatrixXi EV;
  MatrixXi FE;
  vector<list<int> > EF;
  igl::edge_topology_non_manifold(F, EV, FE, EF);

  const int nV = V.rows();
  const int nE = EV.rows();
  const int nF = F.rows();

  // Add mid points if edges are longer than threshold.
  std::vector<Matrix<ScalarV, 1, 3> > midV;
  midV.reserve(nE);
  VectorXi eid_to_midV = -1 * VectorXi::Ones(nE);

  for (int eid = 0; eid < nE; ++eid) {
    const auto& v1 = V.row(EV(eid, 0));
    const auto& v2 = V.row(EV(eid, 1));
    const auto length = (v1 - v2).norm();
    if (length > min_edge_length) {
      eid_to_midV[eid] = midV.size();
      midV.push_back(ScalarV(0.5) * (v1 + v2));
    }
  }

  newV.resize(nV + midV.size(), 3);
  newV.topRows(nV) = V;
  for (int i = 0; i < midV.size(); ++i) {
    newV.row(nV + i) = midV[i];
  }

  // Add subdivided faces.
  std::vector<Matrix<ScalarF, 1, 3> > subF;
  subF.reserve(4 * nF);

  for (int fid = 0; fid < F.rows(); ++fid) {
    Vector3i vids, eids, mid_vids = -1 * Vector3i::Ones();
    int count_face_midV = 0;

    for (int i = 0; i < 3; ++i) {
      vids[i] = F(fid, i);
      eids[i] = FE(fid, i);
      if (eid_to_midV[eids[i]] >= 0) {
        mid_vids[i] = nV + eid_to_midV[eids[i]];
        ++count_face_midV;
      }
    }

    if (count_face_midV == 0) {
      // No edge is cut.
      subF.push_back(F.row(fid));
    }
    else if (count_face_midV == 1) {
      //     v0
      //    /|\
      //   / | \
      //  /f0|f1\
      // v1--m0--v2
      for (int i = 0; i < 3; ++i) {
        if (mid_vids[i] >= 0) {
          const int i0 = i, i1 = (i + 1) % 3, i2 = (i + 2) % 3;
          Matrix<ScalarF, 1, 3> f0, f1;
          f0 << vids[i0], vids[i1], mid_vids[i0];
          f1 << vids[i0], mid_vids[i0], vids[i2];
          subF.push_back(f0);
          subF.push_back(f1);
          break;
        }
      }
    }
    else if (count_face_midV == 2) {
      //      v0
      //      / \
      //  m2 /f0 \m1
      //    /-----\
      //   / f1__/ \
      //  /_/___f2__\
      // v1  m0   v2
      for (int i = 0; i < 3; ++i) {
        if (mid_vids[i] < 0) {
          const int i0 = i, i1 = (i + 1) % 3, i2 = (i + 2) % 3;
          Matrix<ScalarF, 1, 3> f0, f1, f2;
          f0 << vids[i0], mid_vids[i2], mid_vids[i1];
          f1 << mid_vids[i2], vids[i1], mid_vids[i1];
          f2 << mid_vids[i1], vids[i1], vids[i2];
          subF.push_back(f0);
          subF.push_back(f1);
          subF.push_back(f2);
          break;
        }
      }
    }
    else if (count_face_midV == 3) {
      //      v0
      //     / \
      //  m2/___\m1
      //   /\   /\
      //  /__\ /__\
      // v1  m0   v2
      Matrix<ScalarF, 1, 3> f;
      for (int i = 0; i < 3; ++i) {
        const int i0 = i, i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        f << vids[i0], mid_vids[i2], mid_vids[i1];
        subF.push_back(f);
      }
      f << mid_vids[0], mid_vids[1], mid_vids[2];
      subF.push_back(f);
    }
  }

  newF.resize(subF.size(), 3);
  for (int i = 0; i < subF.size(); ++i) {
    newF.row(i) = subF[i];
  }
}

template <
    typename MatV,
    typename MatF>
IGL_INLINE void igl::upsample_non_manifold(
    MatV& V,
    MatF& F,
    const float min_edge_length)
{
  const MatV V_copy = V;
  const MatF F_copy = F;
  upsample_non_manifold(V_copy,F_copy,V,F,min_edge_length);
}

#endif
