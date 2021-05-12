// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef MDS_H
#define MDS_H

#include <stdio.h>
#include <vector>
#include <Eigen/Core>


using namespace Eigen;


namespace MDS {

// Usage:
// @D: Input pairwise distance matrix (N x N).
// @X: Output embedding coordinates matrix (N x ndims).
// const int ndims = 2;
// const auto X = MDS::smacof_rre(D, ndims2);


// -- Type definitions -- //
template <typename T>
using MatrixX = Matrix<T, Dynamic, Dynamic>;

template <typename T>
using VectorX = Matrix<T, Dynamic, 1>;

template <typename T>
using RowVectorX = Matrix<T, 1, Dynamic>;


// -- Function declarations -- //
template <typename T>
MatrixX<T> smacof_rre(
		const MatrixX<T>& _D,
		const int _ndims = 2,
		const int _cycles = 5,
		const int _iter = 10,
		const T _atol = 0.0,
		const bool _verbose = true);

template <typename T>
T calc_stress(const MatrixX<T>& _X, const MatrixX<T>& _D);

template <typename T>
T calc_S(const MatrixX<T>& _D, const MatrixX<T>& _D2);

template <typename T>
MatrixX<T> calc_D(const MatrixX<T>& _X);

template <typename T>
MatrixX<T> calc_B(const MatrixX<T>& _D2, const MatrixX<T>& _D);

template <typename T>
MatrixX<T> extrap(const MatrixX<T>& _XX);

template <typename T>
void mgs(const MatrixX<T>& _A, MatrixX<T>& _Q, MatrixX<T>& _R);


// -- Function definitions -- //
template <typename T>
MatrixX<T> smacof_rre(
		const MatrixX<T>& _D,
		const int _ndims,
		const int _cycles,
		const int _iter,
		const T _atol,
		const bool _verbose) {
	// Initialize.
	const MatrixX<T> X0 = MatrixX<T>::Random(_D.rows(), _ndims);

	int iii_ = 0;
	MatrixX<T> X = X0;
	MatrixX<T> D2 = calc_D(X);
	T S = calc_stress(X0, _D);

	while (iii_ < _cycles) {
		// Internal iteration - MDS
		MatrixX<T> XX = MatrixX<T>::Zero(X.size(), _iter);
		MatrixX<T> Z = X;

		for (int iii = 0; iii < _iter; ++iii) {
			MatrixX<T> B_ = calc_B(D2, _D);
			X = B_ * (Z / (T)(_D.rows()));
			XX.col(iii) = Map< VectorX<T> >(X.data(), X.size());
			D2 = calc_D(X);

			S = calc_S(_D, D2);
			Z = X;

			if (_verbose) {
				const double time = 0.0;
				printf(" ...           %4d   %12.3g   %10.3g\n", iii, S, time);
			}

			// Check stopping conditions.
			if (S < _atol) {
				printf("atol=%g reached, exiting\n", _atol);
				return X;
			}
		}

		// Extrapolate.
		VectorX<T> X_vec = extrap(XX);
		const MatrixX<T> X_ = Map< MatrixX<T> >(X_vec.data(), X.rows(), X.cols());

		// Safeguard.
		const T S_ = calc_S(_D, calc_D(X_));
		if (_verbose) {
			printf("Extrap. stress: %12.3g, MDS stress: %12.3g\n", S_, S);
		}

		if (S_ <= S) {
			X = X_;
			S = S_;
		} else if (_verbose) {
			printf("Safeguard: using MDS solution\n");
		}

		if (_verbose) {
			const double time = 0.0;
			printf("%4d   ------------   %12.3g   %10.3g\n",iii_, S, time);
		}

		iii_ = iii_ + 1;
	}

	return X;
}

template <typename T>
T calc_stress(const MatrixX<T>& _X, const MatrixX<T>& _D) {
	const MatrixX<T> D2 = calc_D(_X);
	const T S = calc_S(_D, D2);
	return S;
}

template <typename T>
T calc_S(const MatrixX<T>& _D, const MatrixX<T>& _D2) {
	MatrixX<T> square_diff = (_D - _D2).array().square();
	square_diff.diagonal().setZero();
	const T S = 0.5 * square_diff.sum();
	return S;
}

template <typename T>
MatrixX<T> calc_D(const MatrixX<T>& _X) {
	const int n = _X.rows();
	MatrixX<T> D = MatrixX<T>::Zero(n, n);
	for (int i = 0; i < n; ++i) {
		D.col(i) = (_X.rowwise() - _X.row(i)).rowwise().norm();
	}
	return D;
}

template <typename T>
MatrixX<T> calc_B(const MatrixX<T>& _D2, const MatrixX<T>& _D) {
	MatrixX<T> B = -_D.cwiseQuotient(_D2);
	B = (_D2.array() == 0).select(0, B);
	B.diagonal().setZero();
	B.diagonal() = -B.rowwise().sum();
	return B;
}

template <typename T>
MatrixX<T> extrap(const MatrixX<T>& _XX) {
	const size_t K = _XX.cols() - 1;
	const MatrixX<T> XXK0 = _XX.leftCols(_XX.cols() - 1);
	const MatrixX<T> XXK1 = _XX.rightCols(_XX.cols() - 1);
	const MatrixX<T> dX = XXK1 - XXK0;

	MatrixX<T> Q, R;
	mgs(dX, Q, R);
	const MatrixX<T> Rt = R.transpose();

	// http://igl.ethz.ch/projects/libigl/matlab-to-eigen.html:
	// MATLAB: X = U\(L\b)
	// Eigen:
	// X = b;
	// L.template triangularView<Lower>().solveInPlace(X);
	// U.template triangularView<Upper>().solveInPlace(X);
	VectorX<T> gamma = VectorX<T>::Ones(K);
	Rt.template triangularView<Lower>().solveInPlace(gamma);
	R.template triangularView<Upper>().solveInPlace(gamma);
	gamma = gamma.array() / gamma.sum();

	VectorX<T> x_ =
		(XXK0.array().rowwise() * gamma.transpose().array()).rowwise().sum();
	return x_;
}

template <typename T>
void mgs(const MatrixX<T>& _A, MatrixX<T>& _Q, MatrixX<T>& _R) {
	const int m = _A.rows();
	const int n = _A.cols();
	// We assume that m >= n.
	MatrixX<T> V = _A;
	_R = MatrixX<T>::Zero(n, n);
	for (int i = 0; i < n; ++i) {
		_R(i, i) = V.col(i).norm();
		V.col(i) = V.col(i) / _R(i, i);
		if (i < n - 1) {
      for (int j = i + 1; j < n; ++j) {
				_R(i, j) = V.col(i).transpose() * V.col(j);
				V.col(j) = V.col(j) - _R(i, j) * V.col(i);
			}
		}
	}
	_Q = V;
}

};	// namespace MDS

#endif	// MDS_H

