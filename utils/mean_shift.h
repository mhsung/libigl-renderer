// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef MEAN_SHIFT_H
#define MEAN_SHIFT_H

#include <iostream>
#include <vector>
#include <Eigen/Core>

using namespace Eigen;


namespace MeanShift {

// Usage:
// MatrixXd data;
// // Fill data. Each *row* is a point.
// const double bandwidth = 1.0;
// const auto clusters = MeanShift::mean_shift<double>(
//   data, bandwidth,
// 	 MeanShift::euclidean_distance<double>,
// 	 MeanShift::gaussian_kernel<double>,
// 	 nullptr);


// -- Type definitions -- //
template <typename T>
using MatrixX = Matrix<T, Dynamic, Dynamic>;

template <typename T>
using VectorX = Matrix<T, Dynamic, 1>;

template <typename T>
using RowVectorX = Matrix<T, 1, Dynamic>;


// -- Metric function declarations -- //
template <typename T>
static VectorX<T> euclidean_distance(
		const RowVectorX<T>& _point, const MatrixX<T>& _points);


// -- Kernel function declarations -- //
template <typename T>
static VectorX<T> gaussian_kernel(
		const VectorX<T>& _distances, const T _bandwidth);


// -- Main function declarations -- //
template <typename T>
std::vector< std::vector<int> > mean_shift(
		const MatrixX<T>& _points,
		const T _bandwidth,
		VectorX<T> (*_metric_func)(const RowVectorX<T>&, const MatrixX<T>&),
		VectorX<T> (*_kernel_func)(const VectorX<T>&, const T),
		MatrixX<T> (*_iter_callback_func)(const MatrixX<T>&, int),
		MatrixX<T>* _shifted_points = nullptr,
		const T _epsilon_distance = 1.0E-6,
		const T _epsilon_clustering = 0.1,
		const int _max_iterations = 100);

template <typename T>
RowVectorX<T> shift_point(
		const RowVectorX<T>& _point,
		const MatrixX<T>& _points,
		const T _bandwidth,
		VectorX<T> (*_metric_func)(const RowVectorX<T>&, const MatrixX<T>&),
		VectorX<T> (*_kernel_func)(const VectorX<T>&, const T));

template <typename T>
std::vector< std::vector<int> > cluster(
		const MatrixX<T>& _points,
		VectorX<T> (*_metric_func)(const RowVectorX<T>&, const MatrixX<T>&),
		const T _epsilon_clustering);


// -- Function definitions -- //
template <typename Derived, typename Array>
PlainObjectBase<Derived> slice_rows(
    const PlainObjectBase<Derived>& _mat, const Array& _idxs) {
  const int n = _idxs.size();
  PlainObjectBase<Derived> ret;
  ret.resize(n, _mat.cols());

  int count = 0;
  for (int i = 0; i < n; ++i) {
    const int idx = _idxs[i];
    CHECK_LE(idx, _mat.rows());
    ret.row(count) = _mat.row(idx);
    ++count;
  }

  return ret;
}

template <typename T>
std::vector< std::vector<int> > mean_shift(
		const MatrixX<T> & _points,
		const T _bandwidth,
		VectorX<T> (*_metric_func)(const RowVectorX<T>&, const MatrixX<T>&),
		VectorX<T> (*_kernel_func)(const VectorX<T>&, const T),
		MatrixX<T> (*_iter_callback_func)(const MatrixX<T>&, int),
    MatrixX<T>* _shifted_points,
		const T _epsilon_distance,
		const T _epsilon_clustering,
		const int _max_iterations) {
	const int n_points = _points.rows();

	MatrixX<T> shifted_points = _points;
	std::vector<bool> is_point_converged(n_points, false);

	if (_iter_callback_func != nullptr) {
		_iter_callback_func(shifted_points, 0);
	}

	for (int iter = 1; iter <= _max_iterations; ++iter) {
		T max_shift_distance = 0;

		for(int pid = 0; pid < n_points; ++pid) {
			if (is_point_converged[pid]) continue;

			const RowVectorX<T> new_point = shift_point<T>(
					shifted_points.row(pid), _points, _bandwidth,
					_metric_func, _kernel_func);

			const T shift_distance = _metric_func(
					new_point, shifted_points.row(pid))[0];

			max_shift_distance = std::max(shift_distance, max_shift_distance);
			if(shift_distance < _epsilon_distance) is_point_converged[pid] = true;

			shifted_points.row(pid) = new_point;
		}

		if (_iter_callback_func != nullptr) {
			shifted_points = _iter_callback_func(shifted_points, iter);
		}

		std::cout << "Max. shift distance: " << max_shift_distance << std::endl;
		if (max_shift_distance < _epsilon_distance) {
			std::cout << "Stop." << std::endl;
			break;
		}
	}

  if (_shifted_points != nullptr) (*_shifted_points) = shifted_points;
	return cluster<T>(shifted_points, _metric_func, _epsilon_clustering);
}

template <typename T>
RowVectorX<T> shift_point(
		const RowVectorX<T>& _point,
		const MatrixX<T>& _points,
		const T _bandwidth,
		VectorX<T> (*_metric_func)(const RowVectorX<T>&, const MatrixX<T>&),
		VectorX<T> (*_kernel_func)(const VectorX<T>&, const T)) {
	const VectorX<T> distances = _metric_func(_point, _points);
	const VectorX<T> weights = _kernel_func(distances, _bandwidth);
	const T total_weight = weights.sum();

	const double kZeroTol = 1.0e-6;
	if (total_weight < kZeroTol) return _point;

	const MatrixX<T> weighted_points =
		_points.array().colwise() * weights.array();
	const RowVectorX<T> shifted_point =
		weighted_points.colwise().sum() / total_weight;

	return shifted_point;
}

template <typename T>
std::vector< std::vector<int> > cluster(
		const MatrixX<T>& _points,
		VectorX<T> (*_metric_func)(const RowVectorX<T>&, const MatrixX<T>&),
		const T _epsilon_clustering) {
	std::vector< std::vector<int> > clusters;

	const int n_points = _points.rows();
	for(int pid = 0; pid < n_points; ++pid) {

		T min_cluster_distance = _epsilon_clustering;
		int selected_cid = -1;

		for (int cid = 0; cid < clusters.size(); ++cid) {
			const auto c_pids = clusters[cid];
			const MatrixX<T> c_points = slice_rows(_points, c_pids);
			const T cluster_distance = _metric_func(
					_points.row(pid), c_points).minCoeff();
			if (cluster_distance < min_cluster_distance) {
				min_cluster_distance = cluster_distance;
				selected_cid = cid;
			}
		}

		if (selected_cid < 0 ) {
			// Create a new cluster.
			clusters.emplace_back(std::vector<int>());
			selected_cid = clusters.size() - 1;
		}
		clusters[selected_cid].push_back(pid);
	}

  // Sort by cluster sizes (in decreasing order).
  std::sort(clusters.begin(), clusters.end(),
      [](const std::vector<int>& _a, const std::vector<int>& _b) {
      return _a.size() > _b.size(); });

	return clusters;
}

template <typename T>
VectorX<T> euclidean_distance(
		const RowVectorX<T>& _point, const MatrixX<T>& _points) {
	const VectorX<T> distances = (_points.rowwise() - _point).rowwise().norm();
	return distances;
}

template <typename T>
VectorX<T> gaussian_kernel(
		const VectorX<T>& _distances, const T _bandwidth) {
	const VectorX<T> ret = (_distances.array().square() /
			(-2.0 * _bandwidth * _bandwidth)).array().exp();
	return ret;
}

};	// namespace MeanShift

#endif	// MEAN_SHIFT_H

