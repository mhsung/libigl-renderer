// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_EDGE_TOPOLOGY_NON_MANIFOLD_H
#define IGL_EDGE_TOPOLOGY_NON_MANIFOLD_H

#include <igl/igl_inline.h>

#include <Eigen/Core>
#include <list>
#include <set>
#include <vector>
#include <unordered_map>

namespace igl 
{
  // Initialize Edges and their topological relations (assumes an edge-manifold
  // mesh)
  //
  // Output:
  // EV : #Ex2, Stores the edge description as pair of indices to vertices
  // FE : #Fx3, Stores the Triangle-Edge relation
  // EF : #Exk: Stores the Edge-Triangle relation
template <typename DerivedF>
  IGL_INLINE void edge_topology_non_manifold(
    const Eigen::PlainObjectBase<DerivedF>& F,
    Eigen::MatrixXi& EV,
    Eigen::MatrixXi& FE,
    std::vector<std::list<int> >& EF);
}

//#ifndef IGL_STATIC_LIBRARY
//#  include "edge_topology_non_manifold.cpp"
//#endif


template <typename DerivedF>
IGL_INLINE void igl::edge_topology_non_manifold(
    const Eigen::PlainObjectBase<DerivedF>& F,
    Eigen::MatrixXi& EV,
    Eigen::MatrixXi& FE,
    std::vector<std::list<int> >& EF)
{
  // Assume triangular mesh.
  assert(F.cols() == 3);

  // Construct edge map.
  // edge_map[v1][v2] = (fid, k),
  // where k \in (0, 1, 2) is the order of the edge's opposite vertex in face.
  const int n_faces = F.rows();
  std::unordered_map<int, std::unordered_map<int,
      std::list< std::pair<int, int> > > > edge_map;
  for (int fid = 0; fid < n_faces ; ++fid) {
    for (int i = 0; i < 3; ++i) {
      int f_vid1 = F(fid, (i + 1) % 3);
      int f_vid2 = F(fid, (i + 2) % 3);
      if (f_vid1 < f_vid2) {
        edge_map[f_vid1][f_vid2].push_back(std::make_pair(fid, i));
      } else {
        edge_map[f_vid2][f_vid1].push_back(std::make_pair(fid, i));
      }
    }
  }

  // Count edges.
  int n_edges = 0;
  for (const auto& v1 : edge_map) {
    const int vid1 = v1.first;
    for (const auto& v2 : v1.second) {
      ++n_edges;
    }
  }

  // Record edge information.
  EV = Eigen::MatrixXi(n_edges, 2);
  FE = Eigen::MatrixXi(n_faces, 3);
  EF.clear();
  EF.resize(n_edges);

  int eid = 0;
  for (const auto& v1 : edge_map) {
    const int vid1 = v1.first;
    for (const auto& v2 : v1.second) {
      const int vid2 = v2.first;
      EV(eid, 0) = vid1;
      EV(eid, 1) = vid2;

      for (const auto& f : v2.second) {
        const int fid = f.first;
        const int f_eid = f.second;
        FE(fid, f_eid) = eid;
        EF[eid].push_back(fid);
      }
      ++eid;
    }
  }
};

#endif
