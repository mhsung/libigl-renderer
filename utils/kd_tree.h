// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef KD_TREE_H
#define KD_TREE_H

#include <memory>
#include <Eigen/Core>
#include <utils/google_tools.h>
#include <utils/nanoflann/nanoflann.hpp>

using namespace Eigen;


namespace KdTree {

/*
 * Example:
 * // D = Data points, Q = Query points (in each row).
 * // NOTE: Do not delete D before retrieving closest points.
 * MatrixXd D, Q;
 * KdTree::KdTreeXd tree(P);
 * VectorXi Q_to_P_idxs;
 * VectorXd Q_to_P_sqr_dists;
 * KdTree::find_k_closest_points(tree, Q, &Q_to_P_idxs, &Q_to_P_sqr_dists, 1);
 */

// Reference:
// https://github.com/jlblancoc/nanoflann/blob/master/tests/test_main.cpp

template<typename Derived>
struct KdTree {
  typedef typename Derived::Scalar Scalar;
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<Scalar, KdTree<Derived> >,
      KdTree<Derived> > KdTreeAdaptor;

  const MatrixBase<Derived>& data_pts_;
  KdTreeAdaptor adaptor_;

  // @_data_pts: N-dimensional data points (each row, N = 2, 3, or 4).
  KdTree(const MatrixBase<Derived>& _data_pts)
      : data_pts_(_data_pts),
        adaptor_(_data_pts.cols(), *this,
                 nanoflann::KDTreeSingleIndexAdaptorParams(10)) {
    CHECK(dim() == 2 || dim() == 3 || dim() == 4)
    << "Dimension must be 2, 3, or 4.";
    adaptor_.buildIndex();
  }

  int dim() const { return data_pts_.cols(); }

  // Template functions required in nanoflann.
  inline size_t kdtree_get_point_count() const { return data_pts_.rows(); }

  inline Scalar kdtree_distance(
      const Scalar* p1, const size_t idx2, size_t /* size*/) const {
    const auto& p2 = data_pts_.row(idx2);
    Scalar ret = 0;
    for (int i = 0; i < dim(); ++i) {
      const Scalar diff = p1[i] - p2[i];
      ret += (diff * diff);
    }
    return ret;
  }

  inline Scalar kdtree_get_pt(const size_t idx, int dim) const {
    const auto& p = data_pts_.row(idx);
    return p[dim];
  }

  template<class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb*/ ) const { return false; }
};

typedef KdTree<Eigen::Matrix<float, -1, -1, 0, -1, -1>> KdTreeXf;
typedef KdTree<Eigen::Matrix<double, -1, -1, 0, -1, -1>> KdTreeXd;


// @_tree: k-d Tree.
// @_query_pts: N-dimensional query points (each row, N = 2, 3, or 4).
// @return: Closest point indices.
template<
    typename DerivedT,
    typename DerivedQ,
    typename DerivedI,
    typename DerivedD>
void find_k_closest_points(
    const KdTree<DerivedT>& _tree,
    const MatrixBase<DerivedQ>& _query_pts,
    Eigen::PlainObjectBase<DerivedI>* _ret_idxs,
    Eigen::PlainObjectBase<DerivedD>* _sqr_dists,
    const int max_K) {
  CHECK_NOTNULL(_ret_idxs);
  CHECK_NOTNULL(_sqr_dists);
  CHECK_EQ(_query_pts.cols(), _tree.dim())
  << "Data and query point dimensions do not match.";

  typedef typename DerivedT::Scalar ScalarT;
  typedef typename DerivedD::Scalar ScalarD;
  const int K = std::min(max_K, (int)_tree.kdtree_get_point_count());

  const int num_query = _query_pts.rows();
  _ret_idxs->resize(num_query, K);
  _sqr_dists->resize(num_query, K);

  for (int i = 0; i < num_query; ++i) {
    // Find the closest data point.
    const Matrix<ScalarT, 1, Dynamic> query_pt =
        _query_pts.row(i).template cast<ScalarT>();
    std::vector<size_t> i_ret_idxs(K);
    std::vector<ScalarT> i_sqr_dists(K);
    nanoflann::KNNResultSet<ScalarT> results(K);
    results.init(&i_ret_idxs[0], &i_sqr_dists[0]);
    _tree.adaptor_.findNeighbors(
        results, query_pt.data(), nanoflann::SearchParams(10));

    for (int j = 0; j < K; ++j) {
      (*_ret_idxs)(i, j) = i_ret_idxs[j];
      (*_sqr_dists)(i, j) = (ScalarD)i_sqr_dists[j];
    }
  }
}

}

#endif // KD_TREE_H
