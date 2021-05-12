// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef MRF_POTTS_H
#define MRF_POTTS_H

#include <array>
#include <vector>
#include <Eigen/Core>
#include <utils/google_tools.h>
#include <utils/TRW_S-v1.3/MRFEnergy.h>

using namespace Eigen;


namespace MRFEigen {

typedef enum {TRW_S, BP} Solver;

// @U: Unary terms (U(node, label) = cost).
// @B: Binary terms (B(node1, node2) = cost).
template<typename Derived,typename SType>
VectorXi solve_mrf_potts(
    const Eigen::PlainObjectBase<Derived>& U,
    Eigen::SparseMatrix<SType>& B,
    const Solver& solver,
    const int max_iter = 50) {
  // Copied from 'utils/TRW_S-v1.3/typePotts.h'
  // Reference:
  // https://www.microsoft.com/en-us/download/details.aspx?id=52499

  typedef typename Derived::Scalar Scalar;

  MRFEnergy<TypePotts>* mrf;
  MRFEnergy<TypePotts>::NodeId* nodes;
  MRFEnergy<TypePotts>::Options options;
  TypePotts::REAL energy, lowerBound;

  const int nodeNum = U.rows(); // number of nodes
  const int K = U.cols(); // number of labels
  std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > D(nodeNum);

  mrf = new MRFEnergy<TypePotts>(TypePotts::GlobalSize(K));
  nodes = new MRFEnergy<TypePotts>::NodeId[nodeNum];

  // construct energy
  for (int i = 0; i < nodeNum; ++i) {
    D[i] = U.row(i).transpose();
    nodes[i] = mrf->AddNode(
        TypePotts::LocalSize(), TypePotts::NodeData(D[i].data()));
  }

  for (int i = 0; i < B.outerSize(); ++i) {
    for (typename Eigen::SparseMatrix<SType>::InnerIterator it(B, i); it;
         ++it) {
      CHECK_LT(it.row(), nodeNum);
      CHECK_LT(it.col(), nodeNum);
      mrf->AddEdge(nodes[it.row()], nodes[it.col()],
                   TypePotts::EdgeData(it.value()));
    }
  }

  // Function below is optional - it may help if, for example, nodes are added in a random order
  // mrf->SetAutomaticOrdering();

  /////////////////////// TRW-S algorithm //////////////////////
  options.m_iterMax = max_iter; // maximum number of iterations
  if (solver == TRW_S) {
    mrf->Minimize_TRW_S(options, lowerBound, energy);
  } else if (solver == BP) {
    mrf->Minimize_BP(options, energy);
  } else assert(false);

  // read solution
  VectorXi ret(nodeNum);
  for (int i = 0; i < nodeNum; ++i) {
    ret[i] = mrf->GetSolution(nodes[i]);
  }

  // done
  delete nodes;
  delete mrf;

  return ret;
}

}

#endif // MRF_POTTS_H
