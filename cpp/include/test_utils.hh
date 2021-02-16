#pragma once

#include "nn_utils/matrix.hh"
#include <Eigen/Dense>
#include <vector>

namespace testutils {

	void initializeTensorWithValue(matrix::Matrix M, float value);
	void initializeTensorWithMatrix(matrix::Matrix M, matrix::Matrix M_input);
	void initializeTensorRandomlyInRange(matrix::Matrix M, float min, float max);
	void initializeEachTensorRowWithValue(matrix::Matrix M, std::vector<float> values);
	void initializeEachTensorColWithValue(matrix::Matrix M, std::vector<float> values);
	void setData(matrix::Matrix X, Eigen::MatrixXd the_pnt_nut);
	// Eigen::MatrixXd Matrix2Eigen(matrix::Matrix mat);
	void Matrix2Eigen(matrix::Matrix mat, Eigen::MatrixXd &eig);

}
