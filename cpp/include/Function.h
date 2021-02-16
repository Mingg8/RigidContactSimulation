#ifndef FUNCTION_H_
#define FUNCTION_H_

#define PI_DEFINED 3.1415926535897932384626433832795028841971 
#define gravity 0.0
// #define gravity 9.81
#define max_buf 3000 // for screw
#define MAX_SIZE 10 // if we change num_limit, then also change this

typedef struct {
    MatrixXd input_coeff_, output_coeff_, weight0_, weight1_, weight2_, weight3_,
			 weight4_, weight5_, weight6_, weight7_;
} my_constraint_data;

typedef struct {
    VectorXd x_pre_, mu_coeff_set_;
} my_cost_data;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////// math function ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
extern Matrix3d skew_func(Vector3d w);
extern Vector3d unskew_func(Matrix3d W);
extern Vector3d Rot2XYZ(Matrix3d R);
extern Vector3d Rot2XYZ_improve(Matrix3d R, double set_theta);
extern bool compare_head(Matrix<double, 1, 2>& lhs, Matrix<double, 1, 2>& rhs);
extern Matrix<double, max_buf, 2> sorted_rows_by_head(Matrix<double, max_buf, 2> A, int rows);
Vector3d calc_ori_error(Quaterniond orientation, Quaterniond orientation_d_);
extern void rotmat_to_angleaxis(Matrix3d R, double* angle, Vector3d* axis);

/////////////////////////////////////////////////////////////////////////////
/////////////////////////// txt function ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
extern int txt_count_row_line(string c);
extern MatrixXd txt_read(string c, int row, int col);
extern void txt_write(string c, MatrixXd data, int row, int col);

/////////////////////////////////////////////////////////////////////////////
////////////////////////////// algorithm ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

extern void calc_lambda(Matrix3d A1, Vector3d b1, double theta, double mu, Vector3d* lambda, double* r);
extern void calc_grad(Matrix3d A1, Vector3d b1, Matrix3d A2, Vector3d b2, Vector3d lambda, double mu, double* D);
extern Vector3d TMD(Matrix3d A1, Matrix3d A2, Vector3d b1, Vector3d b2, double mu, int max_iter, double eps_precision, double beta1, double beta2, double beta3);
extern Vector3d TMD_for_C_space(Matrix3d A1, Matrix3d A2, Vector3d b1, Vector3d b2, double mu, int max_iter, double eps_precision, double beta1, double beta2, double beta3);
extern MatrixXd TMD_multi(MatrixXd A1, MatrixXd A2, MatrixXd b1, MatrixXd b2,
	double mu, int max_iter, double eps_precision, double beta1, double beta2, double beta3, int max_iter_for_multi, double gamma_TMD,
	double eps_precision_for_multi, double alpha_min, int num_cp);
extern MatrixXd TMD_multi_for_C_space(MatrixXd A1, MatrixXd A2, MatrixXd b1, MatrixXd b2,
	double mu, int max_iter, double eps_precision, double beta1, double beta2, double beta3, int max_iter_for_multi, double gamma_TMD,
	double eps_precision_for_multi, double alpha_min, int num_cp);

extern void Contact_Force_Generation(MatrixXd M_mat, MatrixXd Mkhat, MatrixXd iMkhat, MatrixXd dpsik, double Tk, double mu_obj, double eps_res,
	int num_cp, MatrixXd pc_set, MatrixXd nc_set, MatrixXd penetration_set, MatrixXd idx_set, MatrixXd idx_knn_set,
	MatrixXd o_maxmin, MatrixXd Y_mat, int p, Matrix3d obj_static_R, Vector3d obj_static_CoM,
	Vector3d xk, Matrix3d Rk, Vector3d vk, Vector3d wk, MatrixXd Fk,
	MatrixXd n_NN, MatrixXd param, int use_kmeans, MatrixXd Fc_pre,
	MatrixXd* Fc,
	MatrixXd* pc_result, MatrixXd* nc_result, int* n_result);

#endif
