#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "utils.hh"
#include "visualization.hh"
#include "Function.h"
#include "../matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
	int N_simul = 1000;
	double dt = 0.02;
	// SIMULATION
	VectorXd X = VectorXd::Zero(6);
	VectorXd V = VectorXd::Zero(6);
	VectorXd Vhat = VectorXd::Zero(6);
	Matrix3d R = Matrix3d::Identity();
	Quaterniond q = Quaterniond(R);
	q.normalize();

	// initialize
	MatrixXd M, C, K;
	M.resize(6, 6);
	C.resize(6, 6);
	K.resize(6, 6);
	M = MatrixXd::Identity(6, 6) * 0.3;
	M.block<3, 3>(3, 3) = Matrix3d::Identity() * 0.01;
	K = MatrixXd::Identity(6, 6) * 100;
	K.bottomRightCorner(3, 3) = Matrix3d::Identity()*10;
	C = MatrixXd::Identity(6, 6) * 20;
	C.bottomRightCorner(3, 3) = Matrix3d::Identity() * 0.2;
	MatrixXd Mhat = 2*M/dt + C + dt/2*K;
	MatrixXd invA = (2*M/dt + C + dt/2*K).inverse();
	X << 0, 0, 0.1, 0, 0, 0;
	VectorXd f_c = VectorXd::Zero(6);
	double hexa_size = 0.05;
	int c_num = 8;
	MatrixXd collision_cand(3, c_num);
	collision_cand << hexa_size, hexa_size, -hexa_size, -hexa_size, hexa_size/2, hexa_size/2, -hexa_size/2, -hexa_size/2,
					hexa_size, -hexa_size, -hexa_size, hexa_size, hexa_size/2, -hexa_size/2, -hexa_size/2, hexa_size/2,
					-hexa_size, -hexa_size, -hexa_size, -hexa_size, -hexa_size, -hexa_size, -hexa_size, -hexa_size;
	double mu = 0.3;

	// Desired trajectory
	MatrixXd Xd(3, N_simul);
	MatrixXd qd(4, N_simul);
	Xd.setZero();
	for (int i = 0; i < N_simul; i++) {
		if (Xd(2, i) > 0.048) {
			Xd(2, i) = 0.1 - 0.0015 * i;
		} else {
			Xd(2, i) = 0.048;
		}
		Quaterniond q = Quaterniond(AngleAxisd(i*0.0005*M_PI, Vector3d::UnitZ()));
		qd(0, i) = q.w();
		qd(1, i) = q.x();
		qd(2, i) = q.y();
		qd(3, i) = q.z();
	}

	MatrixXf contact_points, rotation;
	contact_points.resize(3, N_simul);
	rotation.resize(3, 3*N_simul);
	
	vector<double> f_x;
	vector<double> f_y;
	vector<double> f_z;
	vector<double> t_x;
	vector<double> t_y;
	vector<double> t_z;

	MatrixXd o_maxmin, Y_mat, idx_set, idx_knn_set, n_NN, param, Fc_pre;
	for (int t = 0; t < N_simul; t++) {
		Vector3d e;
		Quaterniond ori_bar_temp = q;
		// if (qd.col(t).dot(Vector4d(q.w(), q.x(), q.y(), q.w())) < 0.0) {
		// 	ori_bar_temp.coeffs() << -ori_bar_temp.coeffs();
		// }
		Matrix3d qd_mat = Matrix3d(Quaterniond(qd(0, t), qd(1, t), qd(2, t), qd(3, t)));
		// cout << qd_mat << endl;
		Matrix3d q_mat = Matrix3d(q);

		// Quaterniond error_tmp_quaternion(ori_bar_temp * qd.inverse());
		Quaterniond error_tmp_quaternion(ori_bar_temp * qd_mat.inverse());
		AngleAxisd error_tmp_quaternion_angle_axis(error_tmp_quaternion);
		e =  error_tmp_quaternion_angle_axis.axis() * error_tmp_quaternion_angle_axis.angle();
		cout << e.transpose() << endl;

		MatrixXd f(6, 1);
		f.block<3, 1>(0, 0) = -K.block<3, 3>(0, 0) * (X.head(3) - Xd.col(t));
		f.block<3, 1>(3, 0) = -R.transpose() * K.block<3, 3>(3, 3) * e;

		MatrixXd coriolis(6, 6);
		coriolis.setZero();
		coriolis.block<3, 3>(3, 3) = -skew_func(M.block<3, 3>(3, 3) * Vhat.tail(3));
		Mhat = 2*M/dt + C + coriolis;
		Mhat.topLeftCorner(3, 3) = Mhat.topLeftCorner(3, 3) + K.topLeftCorner(3, 3) * dt/ 2;
		Mhat.bottomRightCorner(3, 3) = Mhat.bottomRightCorner(3, 3) + R.transpose() *
			K.bottomRightCorner(3, 3) * dt/2 * R; 
		invA = Mhat.inverse();
		// cout << X.transpose() << endl;

		MatrixXd pc_set(3, c_num), nc_set(3, c_num), penetration_set(1, c_num);
		pc_set.setZero();
		nc_set.setZero();
		penetration_set.setZero();
		int p = 0;
		int num_cp = 0;
		// Collision detection
		for (int c = 0; c < c_num; c++) {
			Vector3d pnt = X.head(3) + R * collision_cand.col(c);
			if (pnt(2) < 0) {
				pc_set.col(num_cp) = pnt;
				nc_set.col(num_cp) = Vector3d(0, 0, 1);
				penetration_set(num_cp) = pnt(2);
				num_cp++;
			}
		}

		MatrixXd Fc, pc_result, nc_result;
		int n_result;
		Contact_Force_Generation(M, Mhat, invA, VectorXd::Zero(6), dt, mu, 1e-3,
			num_cp, pc_set, nc_set, penetration_set, idx_set, idx_knn_set,
			o_maxmin, Y_mat, 0, Matrix3d::Identity(), Vector3d::Zero(),
			X, R, V.head(3), V.tail(3), f,
			n_NN, param, 0, Fc_pre, &Fc, &pc_result, &nc_result, &n_result);
		f_c = Fc.col(0);
		f_x.push_back(f_c(0)); f_y.push_back(f_c(1)); f_z.push_back(f_c(2));
		t_x.push_back(f_c(3)); t_y.push_back(f_c(4)); t_z.push_back(f_c(5));
		// cout << f_c.transpose() << endl;

		// Contact force calculation
		Vhat = invA * (2*M*V/dt + f + f_c);
		V = 2*Vhat - V;
		X = X + Vhat*dt;

		Vector3d del_theta = Vhat.tail(3) * dt;
		double theta = del_theta.norm();
		if (theta > 1e-5) {
			Matrix3d skew_exp = Matrix3d::Identity() + sin(theta)/theta * skew_func(del_theta)
				+ (1-cos(theta))/theta * skew_func(del_theta) * skew_func(del_theta);
			R = R * skew_exp;
		}
		q = R;
		q.normalize();
		contact_points.block<3, 1>(0, t) = X.head(3).cast<float>();
		rotation.block<3, 3>(0, 3*t) = R.cast<float>();
		// cout << X.head(3).transpose() << endl;
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	// VISUALIZATION
	int obj_num = 1;
    unique_ptr<Visualization> vis = make_unique<Visualization>(obj_num);
	vis->setView(glm::vec3(0, 0.05, -0.25), glm::vec3(0, 0.05, 0), PI, PI, 30.0f, 0.5f);

	double lastTime = glfwGetTime();
	int nbFrames = 0;

	float floor_y =  0.0;
	vector<glm::vec3> floor = {glm::vec3(-1.0f, floor_y, -1.0f),
					glm::vec3(-1.0f, floor_y, 1.0f),
					glm::vec3(1.0f, floor_y, -1.0f),
					glm::vec3(1.0f, floor_y, -1.0f),
					glm::vec3(-1.0f, floor_y, 1.0f),
					glm::vec3(1.0f, floor_y, 1.0f)};
	vector<glm::vec3> normal_floor = {glm::vec3(0.0f, -1.0f, 0.0f),
		glm::vec3(0.0f, -1.0f, 0.0f),
		glm::vec3(0.0f, -1.0f, 0.0f),
		glm::vec3(0.0f, -1.0f, 0.0f),
		glm::vec3(0.0f, -1.0f, 0.0f),
		glm::vec3(0.0f, -1.0f, 0.0f)};
	vector<glm::vec3> color_floor = {
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f)};

	vector<glm::vec3> spccolor_floor = {
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f)};

///////////////////////////////////////////////////////////////////////////
	float hs = 0.05f;
	vector<glm::vec3> hexa_pnts = {glm::vec3(-hs, -hs, -hs),
					glm::vec3(hs, -hs, -hs),
					glm::vec3(hs, hs, -hs),
					glm::vec3(-hs, hs, -hs),
					glm::vec3(-hs, -hs, hs),
					glm::vec3(hs, -hs, hs),
					glm::vec3(hs, hs, hs),
					glm::vec3(-hs, hs, hs)};

	vector<unsigned short> indices = {0, 1, 2,
									0, 2, 3,
									1, 2, 5,
									2, 5, 6, 
									4, 5, 6,
									4, 6, 7,
									0, 4, 7,
									0, 3, 7,
									0, 1, 5, 
									0, 4, 5,
									2, 3, 6, 3, 6, 7};


	vector<glm::vec3> hexa_normal = {glm::vec3(0.0f, 0.0f, -1.0f),
									glm::vec3(0.0f, 0.0f, -1.0f),
									glm::vec3(1.0f, 0.0f, 0.0f),
									glm::vec3(1.0f, 0.0f, 0.0f),
									glm::vec3(0.0f, 0.0f, 1.0f),
									glm::vec3(0.0f, 0.0f, 1.0f),
									glm::vec3(-1.0f, 0.0f, 0.0f),
									glm::vec3(-1.0f, 0.0f, 0.0f),
									glm::vec3(0.0f, -1.0f, 0.0f),
									glm::vec3(0.0f, -1.0f, 0.0f),
									glm::vec3(0.0f, 1.0f, 0.0f),
									glm::vec3(0.0f, 1.0f, 0.0f)};

	vector<glm::vec3> hexa_color = {
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f),
		glm::vec3(0.3f, 0.3f, 0.3f)};

	vector<glm::vec3> hexa_spccolor = {
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f)};

		vis->setHexa(hexa_pnts, indices, hexa_color, hexa_normal, hexa_spccolor);

///////////////////////////////////////////////////////////////////////////

	vector<glm::vec3> axis = {
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.1f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.1f, 0.0f),
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, 0.1f)
		};
	vector<glm::vec3> color_axis = {
		glm::vec3(1.0f, 0.0f, 0.0f),
		glm::vec3(1.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 1.0f, 0.0f),
		glm::vec3(0.0f, 1.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, 1.0f),
		glm::vec3(0.0f, 0.0f, 1.0f)};

	Matrix3f R_os;
	R_os << 0, 1, 0,
			0, 0, 1,
			1, 0, 0; // opengl / simulation

	vis->setArrays(1);

	glm::mat4 vis_rot = glm::mat4(1.0f);
	vis_rot = glm::rotate(vis_rot, glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
		VectorXd q_ini = VectorXd::Zero(7);


	auto start_sim = chrono::steady_clock::now();
	do{
		// Measure speed
		double currentTime = glfwGetTime();
		float deltaTime = float(currentTime - lastTime);
		lastTime = currentTime;
		vis->mouseCB();

		if ( currentTime - lastTime >= 1.0 ){ // If last prinf() was more than 1sec ago
			// printf and reset
			printf("%f ms/frame\n", 1000.0/double(nbFrames));
			nbFrames = 0;
			lastTime += 1.0;
		}

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glm::vec3 lightPos = glm::vec3(0, 0.2, 0.3);

		{
			Vector3f trans = R_os * contact_points.col(nbFrames);
			Matrix3f opengl_rot = R_os * rotation.block<3, 3>(0, 3*nbFrames) * R_os.transpose();
			AngleAxisf qf = AngleAxisf(opengl_rot);

			glm::mat4 Model = vis_rot * glm::translate(glm::mat4(1.0f), 
				glm::vec3(trans(0), trans(1), trans(2)))* glm::rotate(glm::mat4(1.0f), qf.angle(), 
				glm::vec3(qf.axis().x(), qf.axis().y(), qf.axis().z()));
			vis->drawObjects(Visualization::OBJ::MALE, Model, lightPos);
		}
		{
			glm::mat4 ModelMatrix = glm::mat4(1.0f);
			vis->drawObjects(Visualization::OBJ::FLOOR, ModelMatrix, lightPos, floor, 
				normal_floor, color_floor, spccolor_floor);
		}
		{
			glm::mat4 ModelMatrix = glm::mat4(1.0f);
			vis->drawObjects(Visualization::OBJ::AXIS, ModelMatrix, lightPos, axis, 
				normal_floor, color_axis);
		}

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);

		// Swap buffers
		glfwSwapBuffers(vis->window);
		glfwPollEvents();
		// nbFrames++;
		nbFrames = nbFrames + 1;
        // if (nbFrames%10 == 0) {
        //     cout << nbFrames << endl;
        // }

	} while( glfwGetKey(vis->window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		glfwWindowShouldClose(vis->window) == 0 && nbFrames < N_simul - 1);
	auto end_sim = chrono::steady_clock::now();
	vis->cleanUp();

	plt::named_plot("f_x", f_x);
	plt::named_plot("f_y", f_y);
	plt::named_plot("f_z", f_z);
	plt::named_plot("t_x", t_x);
	plt::named_plot("t_y", t_y);
	plt::named_plot("t_z", t_z);
	plt::legend();
	plt::show();


	return 0;
}