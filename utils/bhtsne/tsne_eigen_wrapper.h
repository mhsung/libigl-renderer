// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef TSNE_EIGEN_WRAPPER_H
#define TSNE_EIGEN_WRAPPER_H

#include <assert.h>
#include <iostream>
#include <numeric>
#include <Eigen/Core>
#include <utils/bhtsne/tsne.h>

using namespace Eigen;


namespace TSNEEigen {
	template <typename Array>
	std::vector<size_t> argsort(const Array& _v) {
		std::vector<size_t> idx(_v.size());
		std::iota(idx.begin(), idx.end(), 0);
		std::sort(idx.begin(), idx.end(),
				[&_v](size_t i1, size_t i2) { return _v[i1] < _v[i2]; });
		return idx;
	}

	MatrixXd tsne_dists(const MatrixXd& _D,
			int ndims = 2, double perplexity = 50.0) {
		const int N = _D.rows();
		assert(_D.cols() == N);
		const int K = static_cast<int>(3 * perplexity);
		if (K + 1 > N) {
			std::cerr << "TSNE error: Perplexity must be less than N/3." << std::endl;
			exit(-1);
		}

		Matrix<unsigned int, Dynamic, Dynamic, RowMajor> D_inds(N, K);
		Matrix<double, Dynamic, Dynamic, RowMajor> D_vals(N, K);

		for (int i = 0; i < N; ++i) {
			const auto idxs = argsort(_D.row(i));
			int count = 0;
			for (int j = 0; count < K; ++j) {
				//if(idxs[j] == i) continue;
				D_inds(i, count) = idxs[j];
				D_vals(i, count) = _D(i, idxs[j]);
				++count;
			}
		}

		const double theta = 0.5;
		int rand_seed = -1;
		const int max_iter = 1000;

		Matrix<double, Dynamic, Dynamic, RowMajor> Y(N, ndims);

		TSNE tsne;
		tsne.run(D_inds.data(), D_vals.data(), N, K, Y.data(), ndims, perplexity,
				theta, rand_seed, false, max_iter);

		return Y;
	}

	MatrixXd tsne_coords(const MatrixXd& _X,
			int ndims = 2, double perplexity = 50.0) {
		const int N = _X.rows();

		MatrixXd D(N, N);
		for (int i = 0; i < N; ++i) {
			// Compute Euclidean distances.
			D.col(i) = (_X.rowwise() - _X.row(i)).rowwise().norm();
		}

		return tsne_dists(D, ndims, perplexity);
	}
};	// namespace TSNE

#endif	// TSNE_EIGEN_WRAPPER_H

