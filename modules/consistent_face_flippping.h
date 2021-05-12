// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_CONSISTENT_FACE_FLIPPING_H
#define IGL_CONSISTENT_FACE_FLIPPING_H

#include <igl/igl_inline.h>
#include <modules/edge_topology_non_manifold.h>

#include <deque>
#include <list>
#include <vector>
#include <Eigen/Core>

namespace igl
{
  template <
      typename DerivedF,
      typename DerivedA>
  IGL_INLINE void consistent_face_flipping(
    const Eigen::PlainObjectBase<DerivedF>& F,
    const Eigen::PlainObjectBase<DerivedA>& FA,
    Eigen::PlainObjectBase<DerivedF>& newF);

  template <
      typename VecA,
      typename MatF>
  IGL_INLINE void consistent_face_flipping(
    const VecA& FA,
    MatF& F);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "consistent_face_flipping.cpp"
//#endif

template <typename VecF>
IGL_INLINE bool is_orientation_consistent(
    const int v1, const int v2,
    const VecF& f1, const VecF& f2)
{
  const bool is_f1_ccw =
      (f1[0] == v1 && f1[1] == v2) ||
      (f1[1] == v1 && f1[2] == v2) ||
      (f1[2] == v1 && f1[0] == v2);
  const bool is_f2_ccw =
      (f2[0] == v1 && f2[1] == v2) ||
      (f2[1] == v1 && f2[2] == v2) ||
      (f2[2] == v1 && f2[0] == v2);
  // Return true if both faces have different orientations.
  return (is_f1_ccw ^ is_f2_ccw);
}

template <typename Container,typename MatF>
IGL_INLINE void flip_faces(
    const Container& IX, MatF& F)
{
  for (const auto& idx : IX) {
    std::swap(F(idx, 1), F(idx, 2));
  }
}

template <
    typename DerivedF,
    typename DerivedA>
IGL_INLINE void igl::consistent_face_flipping(
    const Eigen::PlainObjectBase<DerivedF>& F,
    const Eigen::PlainObjectBase<DerivedA>& FA,
    Eigen::PlainObjectBase<DerivedF>& newF)
{
  // Assume triangular mesh.
  assert(F.cols() == 3);
  typedef typename DerivedA::Scalar ScalarA;

  typedef enum {
    UP_SIDE = 0,
    DOWN_SIDE = 1,
    UNDEFINED = 2
  } Orientation;

  MatrixXi EV;
  MatrixXi FE;
  std::vector<std::list<int> > EF;
  igl::edge_topology_non_manifold(F, EV, FE, EF);

  const int nF = F.rows();
  std::vector<bool> is_face_visited(nF, false);

  // Copy faces.
  newF = F;

  // Process for each manifold face group.
  while (true) {
    // Find unvisited seed face.
    int seed_fid = 0;
    while (seed_fid < nF && is_face_visited[seed_fid]) ++seed_fid;
    if (seed_fid >= nF) break;

    std::vector<Orientation> orientations(nF, UNDEFINED);
    std::deque<int> queue;
    queue.push_back(seed_fid);
    orientations[seed_fid] = UP_SIDE;

    // Propagate to neighbor faces.
    while (!queue.empty()) {
      const int fid = queue.front();
      queue.pop_front();
      assert (orientations[fid] != UNDEFINED);
      if (is_face_visited[fid]) continue;
      else is_face_visited[fid] = true;

      for (int i = 0; i < 3; ++i) {
        const int eid = FE(fid, i);
        // Skip boundary or non-manifold edges.
        if (EF[eid].size() != 2) continue;
        const int n_fid = (EF[eid].front() != fid) ?
                          EF[eid].front() : EF[eid].back();

        if (is_orientation_consistent(
            EV(eid, 0), EV(eid, 1), F.row(fid), F.row(n_fid))) {
          orientations[n_fid] = orientations[fid];
        } else {
          orientations[n_fid] = (orientations[fid] == DOWN_SIDE) ?
                                UP_SIDE : DOWN_SIDE;
        }

        if (!is_face_visited[n_fid]) queue.push_back(n_fid);
      }
    }

    // Compute areas of each orientation faces and flip smaller group.
    std::list<int> up_sides, down_sides;
    ScalarA sum_up_sides_area = 0, sum_down_sides_area = 0;

    for (int fid = 0; fid < nF; ++fid) {
      if (orientations[fid] == UP_SIDE) {
        up_sides.push_back(fid);
        sum_up_sides_area += FA[fid];
      }
      else if (orientations[fid] == DOWN_SIDE) {
        down_sides.push_back(fid);
        sum_down_sides_area += FA[fid];
      }
    }
    if (sum_up_sides_area < sum_down_sides_area) flip_faces(up_sides, newF);
    else flip_faces(down_sides, newF);
  }
};

template <
    typename VecA,
    typename MatF>
IGL_INLINE void igl::consistent_face_flipping(
    const VecA& FA,
    MatF& F)
{
  const MatF F_copy = F;
  return consistent_face_flipping(F_copy,FA,F);
}

#endif
