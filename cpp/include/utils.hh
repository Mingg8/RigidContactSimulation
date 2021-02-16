#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <string>
#include <math.h>
#include <cmath>
#include <chrono>

#include "yaml-cpp/yaml.h"

using namespace Eigen;
using namespace std;

#define PI 3.141592

MatrixXd readCSV(string file, int rows, int cols);
void readCSV_ref(string file, int rows, int cols, MatrixXd &mat);
MatrixXf readCSV_f(string file, int rows, int cols);
MatrixXi readCSV_i(string file, int rows, int cols);
void ismember(int num, VectorXd vec, bool *logi, int *ind);
int reduced_index(int idx, VectorXd vec);
VectorXd relu(VectorXd x);
MatrixXd pinv(MatrixXd mat);
template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}
Matrix3d quat_to_rotmat(Vector4d);
vector<int> unique(vector<Vector3i> vec);
Matrix3d Skew(Vector3d vec);
Matrix3d w_to_rot(Vector3d vec);
void ismember_row(Vector3i index, vector<Vector3i> vec, bool *logi, int *ind);
VectorXd tanh(VectorXd a);

// template <typename T>
vector<int> sort_indexes(const vector<double> &v, int num);
MatrixXd SE3_gen(Matrix3d R, Vector3d p);
MatrixXd mod_DH_T(double a, double d, double alpha, double theta);
vector<MatrixXd> fk_franka(VectorXd q);
VectorXd ik_franka(MatrixXd T_des, VectorXd q_ini, int max_loop_iter);
MatrixXd jaco_franka(VectorXd q);