// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <initializer_list>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <Eigen/Core>
#include <Eigen/LU>
#include <H5Cpp.h>

#include <utils/google_tools.h>

using namespace Eigen;


namespace Utils {


// -- Random color utils -- //

void random_label_rgb_color(
    const int _label, Vector3f* _color);

std::vector<std::string> split_string(
    const std::string& _str, const char delim = ',');


// -- Eigen matrix utils -- //

// @_vec: Input vector.
// @_query: Query value.
// @return: Vector indices with query value.
template <typename T>
VectorXi find(const Matrix<T, Dynamic, 1>& _vec, const T _query);

// @_vec: Input vector.
// @return: Unique element vector.
template <typename T>
Matrix<T, Dynamic, 1> unique(const Matrix<T, Dynamic, 1>& _vec);

// @_vec: Input vector.
// @_unique: (Output) Unique element vector.
// @_counts: (Output) Counts of unique elements.
template <typename T>
void count_occurrences(const Matrix<T, Dynamic, 1>& _vec,
                       Matrix<T, Dynamic, 1>* _unique, VectorXi* _counts);

// @_vec: Input vector.
template <typename Array>
std::vector<size_t> argsort(const Array& _vec, bool decrease = false);

// @_mat: Input matrix.
// @_indices: Row indices to slice.
// @_sub_mat: Sliced sub matrix.
template <typename Derived, typename Array>
PlainObjectBase<Derived> slice_rows(
    const PlainObjectBase<Derived>& _mat, const Array& _idxs);

// @_mat: Input matrix.
// @_indices: Column indices to slice.
// @_sub_mat: Sliced sub matrix.
template <typename Derived, typename Array>
PlainObjectBase<Derived> slice_cols(
    const PlainObjectBase<Derived>& _mat, const Array& _idxs);


// -- Eigen I/O utils -- //

template <typename T>
bool write_sequence(
    const std::string& _filepath, const T& _sequence);

template <typename Derived>
bool read_eigen_matrix_from_file(
    const std::string& _filepath,
    MatrixBase<Derived>* _matrix,
    const char _delimiter = ',');

template <typename Derived>
bool write_eigen_matrix_to_file(
    const std::string& _filepath,
    const MatrixBase<Derived>& _matrix,
    const char _delimiter = ',');

template <typename Derived>
bool read_eigen_matrix_from_binary(
    const std::string& _filepath,
    PlainObjectBase<Derived>* _matrix);

template <typename Derived>
bool write_eigen_matrix_to_binary(
    const std::string& _filepath,
    const PlainObjectBase<Derived>& _matrix);

template <typename Scalar, int Row, int Column, int Order>
bool read_eigen_matrix_from_hdf5(
    const std::string& _filepath,
		const std::string& _matrix_name,
    Matrix<Scalar, Row, Column, Order>* _matrix);

template <typename Scalar, int Row, int Column, int Order>
bool write_eigen_matrix_to_hdf5(
    const std::string& _filepath,
		const std::string& _matrix_name,
    const Matrix<Scalar, Row, Column, Order>& _matrix);


// -- Template function implementation -- //

template <typename T>
VectorXi find(const Matrix<T, Dynamic, 1>& _vec, const T _query) {
  VectorXi ret;

  std::list<int> idxs;
  for (int i = 0; i < _vec.size(); ++i) {
    if (_vec[i] == _query) idxs.push_back(i);
  }

  ret = VectorXi(idxs.size());
  int count = 0;
  for (const auto idx : idxs) {
    ret[count++] = idx;
  }

  return ret;
}

template <typename T>
Matrix<T, Dynamic, 1> unique(const Matrix<T, Dynamic, 1>& _vec) {
  std::set<T> elements;
  for (int i = 0; i < _vec.size(); ++i) {
    elements.insert(_vec[i]);
  }

  Matrix<T, Dynamic, 1> ret(elements.size());
  int count = 0;
  for (const auto& element : elements) {
    ret[count++] = element;
  }

  return ret;
}

template <typename T>
void count_occurrences(const Matrix<T, Dynamic, 1>& _vec,
                       Matrix<T, Dynamic, 1>* _unique, VectorXi* _counts) {
  CHECK_NOTNULL(_unique);
  CHECK_NOTNULL(_counts);
  (*_unique) = unique(_vec);

  const int n = _unique->size();
  (*_counts) = VectorXi::Zero(n);

  for (int i = 0; i < _vec.size(); ++i) {
    for (int k = 0; k < n; ++k) {
      if (_vec[i] == (*_unique)[k]) {
        ++((*_counts)[k]);
        break;
      }
    }
  }
}

template <typename Array>
std::vector<size_t> argsort(const Array& _vec, bool decrease) {
	std::vector<size_t> idx(_vec.size());
	std::iota(idx.begin(), idx.end(), 0);
	if (decrease) {
		std::sort(idx.begin(), idx.end(),
				[&_vec](size_t i1, size_t i2) { return _vec[i1] > _vec[i2];} );
	} else {
		std::sort(idx.begin(), idx.end(),
				[&_vec](size_t i1, size_t i2) { return _vec[i1] < _vec[i2];} );
	}
	return idx;
}

template <typename Derived, typename Array>
Derived slice_rows(const Derived& _mat, const Array& _idxs) {
  const int n = _idxs.size();
  Derived ret;
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

template <typename Derived, typename Array>
Derived slice_cols(const Derived& _mat, const Array& _idxs) {
  const int n = _idxs.size();
  Derived ret;
  ret.resize(_mat.rows(), n);

  int count = 0;
  for (int i = 0; i < n; ++i) {
    const int idx = _idxs[i];
    CHECK_LE(idx, _mat.cols());
    ret.col(count) = _mat.col(idx);
    ++count;
  }

  return ret;
}

template <typename Scalar>
Scalar string_to_number(const std::string& _str) {
  if (_str.size() == 0) return 0;
  std::istringstream sstr(_str);
  Scalar value = 0;
  if (!(sstr >> std::dec >> value)) throw std::invalid_argument(_str);
  return value;
}

template <typename T>
bool write_sequence(
    const std::string& _filepath, const T& _sequence) {
  std::ofstream file(_filepath);
  if (!file.good()) {
    LOG(WARNING) << "Can't write the file: '" << _filepath << "'." << std::endl;
    return false;
  }

  for (auto it = _sequence.begin(); it != _sequence.end(); ++it) {
    file << (*it) << std::endl;
  }

  file.close();
  return true;
}

template <typename Derived>
bool read_eigen_matrix_from_file(
    const std::string& _filepath,
    MatrixBase<Derived>* _matrix,
    const char _delimiter /*= ','*/) {
  CHECK_NOTNULL(_matrix);
  typedef typename Derived::Scalar Scalar;

  std::ifstream file(_filepath);
  if (!file.good()) {
    LOG(WARNING) << "Can't read the file: '" << _filepath << "'";
    return false;
  }

  typedef std::vector<Scalar> StdVector;
  typedef std::unique_ptr<StdVector> StdVectorPtr;
  typedef std::vector<StdVectorPtr> StdMatrix;
  StdMatrix std_matrix;

  std::string line("");
  int num_rows = 0, num_cols = -1;

  for (; std::getline(file, line); ++num_rows) {
    // Stop reading when the line is blank.
    if (line == "") break;
    std::stringstream sstr(line);
    StdVectorPtr vec(new StdVector);

    std::string token("");
    while (std::getline(sstr, token, _delimiter)) {
      // Stop reading when the token is blank.
      if (token == "") break;
      try {
        const Scalar value = string_to_number<Scalar>(token);
        vec->push_back(value);
      }
      catch (std::exception& e) {
        LOG(WARNING) << e.what();
        return false;
      }
    }

    if (num_cols >= 0 && num_cols != vec->size()) {
      LOG(WARNING) << "The number of cols does not match: " <<
                   num_cols << " != " << vec->size();
      return false;
    }

    num_cols = static_cast<int>(vec->size());
    std_matrix.push_back(std::move(vec));
  }

  (*_matrix) = Matrix<Scalar, Dynamic, Dynamic>(
      num_rows, num_cols);
  for (int i = 0; i < num_rows; ++i) {
    for (int j = 0; j < num_cols; ++j) {
      (*_matrix)(i, j) = (*std_matrix[i])[j];
    }
  }

  file.close();
  return true;
}

template <typename Derived>
bool write_eigen_matrix_to_file(
    const std::string& _filepath,
    const MatrixBase<Derived>& _matrix,
    const char _delimiter /*= ','*/) {
  std::ofstream file(_filepath);
  if (!file.good()) {
    LOG(WARNING) << "Can't write the file: '" << _filepath << "'";
    return false;
  }

  const IOFormat csv_format(
      FullPrecision, DontAlignCols, std::string(1, _delimiter));
  file << _matrix.format(csv_format);
  file.close();
  return true;
}

template <typename Derived>
bool read_eigen_matrix_from_binary(
    const std::string& _filepath,
    const PlainObjectBase<Derived>* _matrix) {
  CHECK_NOTNULL(_matrix);

  typedef typename Derived::Scalar Scalar;

  std::ifstream file(_filepath, std::ios::in | std::ios::binary);
  if (!file.good()) {
    LOG(WARNING) << "Can't read the file: '" << _filepath << "'";
    return false;
  }

  int32_t rows = 0, cols = 0;
  file.read((char*) (&rows), sizeof(int32_t));
  file.read((char*) (&cols), sizeof(int32_t));
  _matrix->resize(rows, cols);
  file.read((char*) _matrix->data(), rows * cols * sizeof(Scalar));
  file.close();
  return true;
}

template <typename Derived>
bool write_eigen_matrix_to_binary(
    const std::string& _filepath,
    const PlainObjectBase<Derived>& _matrix) {
  typedef typename Derived::Scalar Scalar;

  std::ofstream file(_filepath,
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.good()) {
    LOG(WARNING) << "Can't write the file: '" << _filepath << "'";
    return false;
  }

  int32_t rows = static_cast<int32_t>(_matrix.rows());
  int32_t cols = static_cast<int32_t>(_matrix.cols());
  file.write((char*) (&rows), sizeof(int32_t));
  file.write((char*) (&cols), sizeof(int32_t));
  file.write((char*) _matrix.data(), rows * cols * sizeof(Scalar));
  file.close();
  return true;
}

template <typename Scalar, int Row, int Column, int Order>
bool read_eigen_matrix_from_hdf5(
    const std::string& _filepath,
		const std::string& _matrix_name,
    Matrix<Scalar, Row, Column, Order>* _matrix) {
  CHECK_NOTNULL(_matrix);

  try {
    H5::H5File file_h5(_filepath, H5F_ACC_RDONLY);
    H5::DataSet data_set = file_h5.openDataSet(_matrix_name);
		H5::DataSpace data_space = data_set.getSpace();

		int ndims = data_space.getSimpleExtentNdims();
		CHECK_EQ(ndims, 2);
		hsize_t dims[ndims];
		data_space.getSimpleExtentDims(dims, NULL);

		Matrix<Scalar, Row, Column, RowMajor> row_major_matrix(
				dims[0], dims[1]);

		H5::DataType data_type = data_set.getDataType();
		if (std::is_same<Scalar, int>::value) {
			CHECK(data_type == H5::PredType::NATIVE_INT) <<
				"The scalar type does not match: (" << typeid(Scalar).name() << ").";
		}	else if (std::is_same<Scalar, float>::value) {
			CHECK(data_type == H5::PredType::NATIVE_FLOAT) <<
				"The scalar type does not match: (" << typeid(Scalar).name() << ").";
		} else if (std::is_same<Scalar, double>::value) {
			CHECK(data_type == H5::PredType::NATIVE_DOUBLE) <<
				"The scalar type does not match: (" << typeid(Scalar).name() << ").";
		} else {
			LOG(ERROR) << "The scalar type does not match: ("
				<< typeid(Scalar).name() << ").";
		}

		data_set.read(row_major_matrix.data(), data_type);
		(*_matrix) = row_major_matrix;
  } catch (H5::Exception error) {
    error.printError();
    LOG(WARNING) << "Can't read the file: '" << _filepath << "'";
		return false;
  }
  return true;
}

template <typename Scalar, int Row, int Column, int Order>
bool write_eigen_matrix_to_hdf5(
    const std::string& _filepath,
		const std::string& _matrix_name,
    const Matrix<Scalar, Row, Column, Order>& _matrix) {
	const int ndims = 2;
	hsize_t dims[ndims];
	dims[0] = _matrix.rows();
	dims[1] = _matrix.cols();
	H5::DataSpace data_space(ndims, dims);

	H5::DataType data_type;
	if (std::is_same<Scalar, int>::value) {
		data_type = H5::PredType(H5::PredType::NATIVE_INT);
	}	else if (std::is_same<Scalar, float>::value) {
		data_type = H5::PredType(H5::PredType::NATIVE_FLOAT);
	} else if (std::is_same<Scalar, double>::value) {
		data_type = H5::FloatType(H5::PredType::NATIVE_DOUBLE);
	} else {
		LOG(ERROR) << "The scalar type is not supported: ("
			<< typeid(Scalar).name() << ").";
	}

	const Matrix<Scalar, Row, Column, RowMajor> row_major_matrix =
		_matrix;

  try {
    H5::H5File file_h5(_filepath, H5F_ACC_TRUNC);
		H5::DataSet data_set = file_h5.createDataSet(
				_matrix_name, data_type, data_space);
    data_set.write(row_major_matrix.data(), data_type);
  } catch (H5::Exception error) {
    error.printError();
    LOG(WARNING) << "Can't write the file: '" << _filepath << "'";
		return false;
  }

  return true;
}

}

#endif // UTILS_H
