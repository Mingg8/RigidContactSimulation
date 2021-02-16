// Include Files
#include "utils.hh"
#include "Function.h"

// general functions
Matrix3d skew_func(Vector3d w)
{
	Matrix3d W_temp;
	W_temp << 0.0, -w(2), w(1),
		w(2), 0.0, -w(0),
		-w(1), w(0), 0.0;
	return W_temp;
}
Vector3d unskew_func(Matrix3d W)
{
	Vector3d w_temp;
	w_temp << -W(1, 2), W(0, 2), -W(0, 1);
	return w_temp;
}


Vector3d Rot2XYZ(Matrix3d R)
{
	Vector3d y;
	y(0, 0) = atan2(-R(1, 2),R(2, 2));
	y(1, 0) = asin(R(0, 2));
	y(2, 0) = atan2(-R(0, 1),R(0, 0));

	return y;
}

Vector3d Rot2XYZ_improve(Matrix3d R, double set_theta)
{
	Vector3d y;
	y(0) = atan2(-R(1, 2),R(2, 2));
	y(1) = asin(R(0, 2));
	y(2) = atan2(-R(0, 1),R(0, 0));

	int set_n = floor((set_theta + PI_DEFINED)/(2.0 * PI_DEFINED));
	double val1, val2, val3;
	val1 = y(2) + 2.0 * PI_DEFINED * set_n;
	val2 = y(2) + 2.0 * PI_DEFINED * (set_n - 1);
	val3 = y(2) + 2.0 * PI_DEFINED * (set_n + 1);
	if (abs(val1 - set_theta) < abs(val2 - set_theta))
	{
		if (abs(val1 - set_theta) < abs(val3 - set_theta))
		{
			y(2) = val1;
		}
		else{
			y(2) = val3;
		}
	}
	else{
		if (abs(val2 - set_theta) < abs(val3 - set_theta))
		{
			y(2) = val2;
		}
		else{
			y(2) = val3;
		}
	}
	return y;
}

bool compare_head(Matrix<double, 1, 2>& lhs, Matrix<double, 1, 2>& rhs)
{
	return lhs(0, 0) < rhs(0, 0);
}

Matrix<double, max_buf, 2> sorted_rows_by_head(Matrix<double, max_buf, 2> A, int rows)
{
	vector< Matrix<double, 1, 2> > vec;
	for (int64_t i = 0; i < rows; ++i)
		vec.push_back(A.row(i));

	sort(vec.begin(), vec.end(), &compare_head);

	for (int64_t i = 0; i < rows; ++i)
		A.row(i) = vec[i];
	vec.clear();
	vec.shrink_to_fit();

	return A;
}


Vector3d calc_ori_error(Quaterniond orientation, Quaterniond orientation_d_)
{
	Vector3d ori_error;

	Matrix3d R, Rd, Re;
	R = orientation.toRotationMatrix();
	Rd = orientation_d_.toRotationMatrix();
	Re = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

	ori_error(0) = Re(2, 1);
	ori_error(1) = -Re(2, 0);
	ori_error(2) = Re(1, 0);

	ori_error = R * ori_error;

	// Re = R * Rd.transpose();
	// AngleAxisd angle_axis(Re);
	// ori_error = angle_axis.angle() * angle_axis.axis();

	return ori_error;
}

extern void rotmat_to_angleaxis(Matrix3d R, double* angle, Vector3d* axis)
{
	double trR;
	trR = R(0, 0) + R(1, 1) + R(2, 2);
	if (trR > 3.0)
	{
		trR = 3.0;
	}
	else if (trR < -1.0)
	{
		trR = -1.0;
	}
	double angle_tmp;
	Vector3d axis_tmp;
	angle_tmp = acos((trR - 1.0)/2.0);
	if (abs(angle_tmp) <= 1e-6)
	{
		axis_tmp << 0.0, 0.0, 1.0;
		angle_tmp = 0.0;
	}
	else{
		axis_tmp(0) = R(2, 1) - R(1, 2);
		axis_tmp(1) = R(0, 2) - R(2, 0);
		axis_tmp(2) = R(1, 0) - R(0, 1);
		axis_tmp.normalize();
	}

	*angle = angle_tmp;
	*axis = axis_tmp;
}









int txt_count_row_line(string c)
{
	int N = 0;
	ifstream file;
	string line;
	file.open(c.c_str());
	while (getline(file, line))	N++;
	file.close();
	cout << "Row Line of txt (" << c << "): " << N << endl;
	return N;
}

MatrixXd txt_read(string c, int row, int col)
{
	MatrixXd data(row, col);
	ifstream file;
	file.open(c.c_str());
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			file >> data(i, j);
		}
	}
	file.close();
	return data;
}

void txt_write(string c, MatrixXd data, int row, int col)
{
	ofstream file_out(c.c_str());
	for (int kk = 0; kk < row; kk++)
	{
		for (int kk2 = 0; kk2 < col; kk2++)
		{
			file_out << data(kk, kk2) << "\t";
		}
		file_out << endl;
	}
	file_out.close();
}







MatrixXd calcContactTangent_2D(Vector3d cv_normal, Vector3d ref_vec)
{
	Vector3d m_ref_vec;
	
	if (ref_vec.norm() <= 1e-15)
	{
		m_ref_vec.setRandom();
	}
	else
	{
		m_ref_vec = ref_vec / ref_vec.norm() * 10.0;
	}
	m_ref_vec.normalize();
	
	cv_normal.normalize();

	Vector3d ref_tan_vec1 = cv_normal.cross(m_ref_vec);
	
	double norm_v = ref_tan_vec1.norm();
	if (norm_v <= 1e-3)
	{
		// normal direction and reference are parellel
		Vector3d temp_vn;
		temp_vn << 0.0, 1.0, 0.0;
		ref_tan_vec1 = cv_normal.cross(temp_vn);
		norm_v = ref_tan_vec1.norm();
		if (norm_v <= 1e-3)
		{
			temp_vn << 1.0, 0.0, 0.0;
			ref_tan_vec1 = cv_normal.cross(temp_vn);
		}
	}
	

	ref_tan_vec1.normalize();

	MatrixXd cv_tangent(3, 2);
	cv_tangent.block<3, 1>(0, 0) = ref_tan_vec1;
	Vector3d temp;
	temp = cv_normal.cross(ref_tan_vec1);
	temp.normalize();
	cv_tangent.block<3, 1>(0, 1) = temp;

	return cv_tangent;
}


void calc_lambda(Matrix3d A1, Vector3d b1, double theta, double mu, Vector3d* lambda, double* r)
{
	double r_tmp = 0.0;
	if (b1(0) >= 0.0)
	{
		r_tmp = -b1(0) / (- A1(0, 0) / mu + A1(0, 1) * cos(theta) + A1(0, 2) * sin(theta));
		*r = r_tmp;
		(*lambda)(0) = -r_tmp/mu;
		(*lambda)(1) = r_tmp * cos(theta);
		(*lambda)(2) = r_tmp * sin(theta);
	}
	else{
		r_tmp = -b1(0) / ( A1(0, 0) / mu + A1(0, 1) * cos(theta) + A1(0, 2) * sin(theta) );
		*r = r_tmp;
		(*lambda)(0) = r_tmp/mu;
		(*lambda)(1) = r_tmp * cos(theta);
		(*lambda)(2) = r_tmp * sin(theta);
	}
}
void calc_grad(Matrix3d A1, Vector3d b1, Matrix3d A2, Vector3d b2, Vector3d lambda, double mu, double* D)
{
	Vector3d gradh1, gradh2, eta;
	gradh1 = A1.block<1, 3>(0, 0).transpose();
	gradh2(0) = -2.0 * mu * mu * lambda(0);
	gradh2(1) = 2.0 * lambda(1);
	gradh2(2) = 2.0 * lambda(2);
	eta = gradh1.cross(gradh2);

	MatrixXd D_tmp;
	MatrixXd mat1(3, 2);
	mat1 << 0.0, 0.0,
			1.0, 0.0,
			0.0, 1.0;
	// D_tmp = ( lambda.transpose() * mat1 * A2.block<2, 3>(1, 0) 
	// 			+ lambda.transpose() * A2.block<2, 3>(1, 0).transpose() * mat1.transpose()
	// 			+ (mat1 * b2.block<2, 1>(1, 0)).transpose() 
	// 			+ lambda.transpose() ) * eta;
	D_tmp = ( lambda.transpose() * mat1 * A2.block<2, 3>(1, 0) 
				+ lambda.transpose() * A2.block<2, 3>(1, 0).transpose() * mat1.transpose()
				+ (mat1 * b2.block<2, 1>(1, 0)).transpose()) * eta;
	// D_tmp = (2.0 * lambda.transpose() * A2 + b2.transpose()) * eta;
	*D = D_tmp(0, 0);
}

Vector3d TMD(Matrix3d A1, Matrix3d A2, Vector3d b1, Vector3d b2, double mu, int max_iter, double eps_precision, double beta1, double beta2, double beta3)
{
	Vector3d lambda, lambda_tmp, lambda_p, lambda0_v0, lambda_delta;
	double r, theta, theta_p, theta_tmp, theta_delta;
	double D0, D0_new, Dp, D_delta;
	lambda.setZero();
	lambda_tmp.setZero();
	lambda_p.setZero();
	lambda_delta.setZero();
	r = 0.0;
	theta = 0.0;
	theta_p = 0.0;
	theta_tmp = 0.0;
	theta_delta = 0.0;
	D0 = 0.0;
	D0_new = 0.0;
	Dp = 0.0;
	D_delta = 0.0;

	Vector3d grad, gradh1, gradh2;
	grad.setZero();
	gradh1.setZero();
	gradh2.setZero();

	lambda0_v0 = -A1.ldlt().solve(b1);
	// lambda0_v0 = -A1.inverse() * b1;

	// if (b1(0) > 0.0 || lambda0_v0(0) < 0.0) {
	if (b1(0) > 0.0) {
	}
	else if (mu * abs(lambda0_v0(0)) >= sqrt(lambda0_v0(1) * lambda0_v0(1) + lambda0_v0(2) * lambda0_v0(2)))
	{
		lambda = lambda0_v0;
	}
	else
	{
		theta = atan2(lambda0_v0(2), lambda0_v0(1));
		calc_lambda(A1, b1, theta, mu, &lambda, &r);
		calc_grad(A1, b1, A2, b2, lambda, mu, &D0);
		
		double alpha;
		if (D0 > 0.0)
		{
			alpha = - beta1;
		}
		else
		{
			alpha = beta1;			
		}
		

		bool true_index = true;
		int count = 0;

		while (true_index)
		{
			theta_p = theta;
			lambda_p = lambda;
			theta = theta + alpha;
			calc_lambda(A1, b1, theta, mu, &lambda, &r);

			gradh1 = A1.block<1, 3>(0, 0).transpose();
			gradh2(0) = -2.0 * mu * mu * lambda(0);
			gradh2(1) = 2.0 * lambda(1);
			gradh2(2) = 2.0 * lambda(2);

			grad = (gradh1.cross(gradh2)).cross(gradh1);
			
			MatrixXd mat_tmp;
			mat_tmp = grad.transpose() * (lambda - lambda0_v0);
			if (mat_tmp(0, 0) > 0.0 || r < 0.0)
			// mat_tmp = grad2.transpose() * (lambda - lambda0_v0);
			// if (mat_tmp(0, 0) > 0.0 || r < 0.0)
			{
				alpha = beta2 * alpha;
				theta = theta_p;
				lambda = lambda_p;
			}
			else
			{
				calc_grad(A1, b1, A2, b2, lambda, mu, &D0_new);
				if (D0_new * D0 > 0.0)
				{
					alpha = beta3 * alpha;
				}
				else
				{
					if (theta > theta_p)
					{
						theta_tmp = theta;
						lambda_tmp = lambda;
						theta = theta_p;
						lambda = lambda_p;
						theta_p = theta_tmp;
						lambda_p = lambda_tmp;
					}
					calc_grad(A1, b1, A2, b2, lambda_p, mu, &Dp);
					true_index = false;
				}
			}

			count += 1;
			if (count >= max_iter)
			{
				true_index = false;

				if (D0 > 0.0)
				{
					theta = -PI_DEFINED;
					theta_p = atan2(lambda0_v0(2), lambda0_v0(1));
				}
				else
				{
					theta = atan2(lambda0_v0(2), lambda0_v0(1));
					theta_p = PI_DEFINED;
				}

				calc_lambda(A1, b1, theta, mu, &lambda, &r);
				calc_lambda(A1, b1, theta_p, mu, &lambda_p, &r);
				calc_grad(A1, b1, A2, b2, lambda_p, mu, &Dp);
			}			
		}

		// double theta_p_origin = theta_p;
		// double theta_origin = theta;
		// double Dp_origin = Dp;

		for (int i = 0; i < max_iter; i++)
		{
			theta_delta = 0.5 * (theta + theta_p);
			calc_lambda(A1, b1, theta_delta, mu, &lambda_delta, &r);
			calc_grad(A1, b1, A2, b2, lambda_delta, mu, &D_delta);

			if (D_delta * Dp > 0.0)
			{
				theta_p = theta_delta;
				lambda_p = lambda_delta;
				Dp = D_delta;
			}
			else
			{
				theta = theta_delta;
				lambda = lambda_delta;
			}

			// if (abs(D_delta) < eps_precision || i == max_iter - 1)
			if ((lambda_p - lambda).norm() < eps_precision || i == max_iter - 1)
			{
				// cout << "========" << endl;
				// cout << i << endl;
				// cout << D_delta << endl;
				// cout << lambda.transpose() << endl;
				// cout << lambda_p.transpose() << endl;
				// cout << theta_p_origin << " " << theta_origin << " " << Dp_origin << endl;
				// lambda = (lambda_p + lambda)/2.0;
				break;
			}
		}


	}
	return lambda;

}

MatrixXd TMD_multi(MatrixXd A1, MatrixXd A2, MatrixXd b1, MatrixXd b2,
	double mu, int max_iter, double eps_precision, double beta1, double beta2, double beta3, int max_iter_for_multi, double gamma_TMD,
	double eps_precision_for_multi, double alpha_min, int num_cp)
{

	double decay_rate = 1.0;
	MatrixXd lambda(3 * num_cp, 1);
	MatrixXd lambda_pre(3 * num_cp, 1);
	MatrixXd v_final(3 * num_cp, 1);
	lambda.setZero();
	Vector3d lambda_temp;
	MatrixXd A1_temp1, A2_temp1, A1_temp2, A2_temp2;
	Vector3d b1_edit, b2_edit;

	int k_save = 0;
	for (int k = 0; k < max_iter_for_multi; k++)
	{
		lambda_pre = lambda;

		for (size_t i = 0; i < num_cp; i++)
		{
			A1_temp1 = A1.block(3 * i, 0, 3, 3 * num_cp);
			A1_temp2 = A1.block(3 * i, 3 * i, 3, 3);
			b1_edit = b1.block(3 * i, 0, 3, 1) + A1_temp1 * lambda - A1_temp2 * lambda.block(3 * i, 0, 3, 1);

			A2_temp1 = A2.block(3 * i, 0, 3, 3 * num_cp);
			A2_temp2 = A2.block(3 * i, 3 * i, 3, 3);
			b2_edit = b2.block(3 * i, 0, 3, 1) + A2_temp1 * lambda - A2_temp2 * lambda.block(3 * i, 0, 3, 1);

			lambda_temp = TMD(A1_temp2, A2_temp2, b1_edit, b2_edit, mu, max_iter, eps_precision, beta1, beta2, beta3);

			lambda_temp = decay_rate * lambda_temp + (1.0 - decay_rate) * lambda.block(3 * i, 0, 3, 1);
			lambda(3 * i, 0) = lambda_temp(0);
			lambda(3 * i + 1, 0) = lambda_temp(1);
			lambda(3 * i + 2, 0) = lambda_temp(2);
		}
		decay_rate = alpha_min + gamma_TMD * (decay_rate - alpha_min);

		if (lambda.norm() <= 0.001)
		{			
			// for (size_t i = 0; i < num_cp; i++)
			// {
			// 	b1(3 * i, 0) = b1(3 * i, 0) - 0.002;
			// 	b2(3 * i, 0) = b2(3 * i, 0) - 0.002;
			// }
			// if (k > 10)
			// {
			// 	break;
			// }
			k_save = k;
			break;
		}
		else{
			if ((lambda - lambda_pre).norm()/lambda.norm() <= eps_precision_for_multi)
			{
				// break;
				MatrixXd v_final;
				v_final = A1 * lambda + b1;
				// cout << "============" << endl;
				// cout << (lambda - lambda_pre).norm()/lambda.norm() << endl;
				// cout << v_final.transpose() << endl;
				// cout << k << endl;
				
				bool v_check = true;
				for (int i = 0; i < num_cp; i++)
				{
					if (v_final(3 * i, 0) < -1e-5)
					{
						v_check = false;
					}
				}
				if (v_check)
				{
					k_save = k;
					break;
				}
			}
		}
		k_save = k;
	}
	// cout << k_save << endl;
	return lambda;
}

void Contact_Force_Generation(MatrixXd M_mat, MatrixXd Mkhat, MatrixXd iMkhat, MatrixXd dpsik, double Tk, double mu_obj, double eps_res,
	int num_cp, MatrixXd pc_set, MatrixXd nc_set, MatrixXd penetration_set, MatrixXd idx_set, MatrixXd idx_knn_set,
	MatrixXd o_maxmin, MatrixXd Y_mat, int p, Matrix3d obj_static_R, Vector3d obj_static_CoM,
	Vector3d xk, Matrix3d Rk, Vector3d vk, Vector3d wk, MatrixXd Fk,
	MatrixXd n_NN, MatrixXd param, int use_kmeans, MatrixXd Fc_pre,
	MatrixXd* Fc,
	MatrixXd* pc_result, MatrixXd* nc_result, int* n_result)
{	
	MatrixXd Vk(6, 1), Bk_origin(6, 1);

	Vk.block<3, 1>(0, 0) = vk;
	Vk.block<3, 1>(3, 0) = wk;
	Bk_origin = 2.0 * M_mat / Tk * Vk - dpsik + Fk;

	MatrixXd Fc_temp(6, 1);
	Fc_temp.setZero();

	if (num_cp > 0)
	{
		int max_iter = 20;
		double eps_precision = 1e-5;
		int max_iter_for_multi = 50;
		double eps_precision_for_multi = 0.01;
		double beta1 = PI_DEFINED / 6.0;
		double beta2 = 0.9;
		double beta3 = 1.1;
		double gamma_TMD = 0.99;
		double alpha_min = 0.5;

		Vector3d xk_wrt_obj_static, vk_wrt_obj_static, wk_wrt_obj_static;
		Matrix3d Rk_wrt_obj_static;
		MatrixXd Fextk_wrt_obj_static(6, 1);
		MatrixXd pc_cluster, nc_cluster, nc_origin, nc_origin_NN, pc, nc, penetration_final;
		pc_cluster.setZero();
		nc_cluster.setZero();
		nc_origin.setZero();
		nc_origin_NN.setZero();
		penetration_final.setZero();
		pc.setZero();
		nc.setZero();
		int N_contact;

        pc_cluster = pc_set;
        nc_cluster = nc_set;
        nc_origin = nc_set;
        penetration_final = penetration_set;
        N_contact = num_cp;

		MatrixXd mat_ones(1, N_contact);
		mat_ones.setOnes();
		pc = obj_static_CoM * mat_ones + obj_static_R * pc_cluster;
		nc = obj_static_R * nc_cluster;
		nc_origin = obj_static_R * nc_origin;
		for (int i = 0; i < N_contact; i++)
		{
			nc.block<3, 1>(0, i).normalize();
			nc_origin.block<3, 1>(0, i).normalize();
		}

		*pc_result = pc;
		*nc_result = nc;
		*n_result = N_contact;

		
		// for contact algorithm
		MatrixXd NDk(6, 3 * N_contact);
		MatrixXd Ad_mat(6, 3);
		NDk.setZero();
		Ad_mat.setZero();
		Ad_mat(0, 0) = 1.0;
		Ad_mat(1, 1) = 1.0;
		Ad_mat(2, 2) = 1.0;

		for (size_t j = 0; j < N_contact; j++)
		{
			Ad_mat.block<3, 3>(3, 0) = Rk.transpose() * skew_func(pc.block<3, 1>(0, j) - xk);
			NDk.block<6, 1>(0, 3 * j) = Ad_mat * nc.block<3, 1>(0, j);
			NDk.block<6, 2>(0, 3 * j + 1) = Ad_mat * calcContactTangent_2D(nc.block<3, 1>(0, j), vk);
		}

		MatrixXd A1, A2, b1, b2;
		MatrixXd Bk(6, 1);
		Bk = Bk_origin - 0.5 * Mkhat * Vk;
		A1 = NDk.transpose() * 2.0 * iMkhat * NDk;
		b1 = NDk.transpose() * 2.0 * iMkhat * Bk;
		A2 = NDk.transpose() * iMkhat * NDk;
		b2 = NDk.transpose() * iMkhat * Bk_origin;
		
		double eps_add = 1e-4;
		double eps_multi = 0.001;

		for (size_t j = 0; j < N_contact; j++)
		{
			MatrixXd cos_theta(1, 1);
			cos_theta = nc_origin.block<3, 1>(0, j).transpose() * nc.block<3, 1>(0, j);
			double penet = penetration_final(0, j)/cos_theta(0, 0);

			// penetration is minus
			if (penet <= -eps_add)
			{
				b1(3 * j, 0) = b1(3 * j, 0) - abs(penet + eps_add) * eps_multi /  Tk;
			}
		}
		
		// contact algorithm
		MatrixXd lambda_c(N_contact * 3, 1);

		lambda_c = TMD_multi(A1, A2, b1, b2,
			mu_obj, max_iter, eps_precision, beta1, beta2, beta3, max_iter_for_multi, gamma_TMD,
			eps_precision_for_multi, alpha_min, N_contact);
		Fc_temp = NDk * lambda_c;

	}
	*Fc = Fc_temp;
}


