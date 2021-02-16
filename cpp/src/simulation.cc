#include "simulation.hh"

Simulation::Simulation() {
}

void Simulation::setMatrices() {
    cout << "set matrices" << endl;
    end_time = config.end_time;
    dt = config.dt;

    if (config.control == CONST_FORCE) {
        N_simul = floor(end_time / dt);
    } else if (config.control == IMPEDANCE) {
        N_simul = config.N_simul;
    }

    for (int m = 0; m < obj_num; m++) {
        MatrixXd tmp = 2/dt * Mass[m] + Damp[m] + dt/2 * K[m];
        Mhat.push_back(tmp);
        iMhat.push_back(tmp.inverse());
    }

    int contact_size = config.clustering_num;
    if (config.clustering_num == -1) {
        contact_size = 20;
    }
    contact_points.resize(contact_size*3*2, N_simul);
    contact_points.setZero();
    contact_normals.resize(contact_size*3*2, N_simul);
    contact_normals.setZero();
}


void Simulation::loadMatrices() {
    cout << "load matrix" << endl;
    MatrixXd tmp;
    tmp.resize(0, 0);

    if (config.model == NONE) {
        readCSV_ref(config.filepath + "node_const_1.csv", 1, config.const_num[0], tmp);
        for (int i = 0; i < config.const_num[0]; i++) {
            tmp(0, i) = tmp(0, i) - 1;
        }
        node_const.push_back(tmp);

        readCSV_ref(config.filepath + "node_const_2.csv", 1, config.const_num[1], tmp);
        for (int i = 0; i < config.const_num[1]; i++) {
            tmp(0, i) = tmp(0, i) - 1;
        }
        node_const.push_back(tmp);

        readCSV_ref(config.filepath + "node_pnt_const_1.csv", config.const_num[0], 3, tmp);
        node_pnt_const.push_back(tmp);
        readCSV_ref(config.filepath + "node_pnt_const_2.csv", config.const_num[1], 3, tmp);
        node_pnt_const.push_back(tmp);

        for (int i = 0; i < obj_num; i++) {
            N_equ.push_back(3*(config.node_size[i] - config.const_num[i]));
        }

    } else {
        node_const.push_back(tmp);
        node_const.push_back(tmp);

        readCSV_ref(config.filepath + "node_const_3.csv", 1, config.const_num[2], tmp);
        for (int i = 0; i < config.const_num[2]; i++) {
            tmp(0, i) = tmp(0, i) - 1;
        }
        node_const.push_back(tmp);
        readCSV_ref(config.filepath + "node_const_4.csv", 1, config.const_num[3], tmp);
        for (int i = 0; i < config.const_num[3]; i++) {
            tmp(0, i) = tmp(0, i) - 1;
        }
        node_const.push_back(tmp);

        tmp.resize(0, 3);
        node_pnt_const.push_back(tmp);
        node_pnt_const.push_back(tmp);
        readCSV_ref(config.filepath + "node_pnt_const_3.csv", config.const_num[2], 3, tmp);
        node_pnt_const.push_back(tmp);
        readCSV_ref(config.filepath + "node_pnt_const_4.csv", config.const_num[3], 3, tmp);
        node_pnt_const.push_back(tmp);

        MatrixXd H00, H11, H22, H33, H02, H13, H20, H31;
        int const_nodes[2]   = {config.const_num[0], config.const_num[1]};
        for (int i = 0; i < 4; i++) {
            N_equ.push_back(3*(config.node_size[i] - config.const_num[i]));
        }
        int coup_num[2] = {config.coupling_num[0]*3, config.coupling_num[1]*3};

        H00 = readCSV(config.filepath + "H11.csv", coup_num[0], N_equ[0]);
        H11 = readCSV(config.filepath + "H22.csv", coup_num[1], N_equ[1]);
        H22 = readCSV(config.filepath + "H33.csv", coup_num[0], N_equ[2]);
        H33 = readCSV(config.filepath + "H44.csv", coup_num[1], N_equ[3]);
        H02 = readCSV(config.filepath + "H13.csv", coup_num[0], N_equ[2]);
        H13 = readCSV(config.filepath + "H24.csv", coup_num[1], N_equ[3]);
        H20 = readCSV(config.filepath + "H31.csv", coup_num[0], N_equ[0]);
        H31 = readCSV(config.filepath + "H42.csv", coup_num[1], N_equ[1]);

        MatrixXd tmp_mat;
        vector<MatrixXd> H_vec1 = {H00, tmp_mat, H02, tmp_mat};
        H.push_back(H_vec1);
        vector<MatrixXd> H_vec2 = {tmp_mat, H11, tmp_mat, H13};
        H.push_back(H_vec2);
        vector<MatrixXd> H_vec3 = {H20, tmp_mat, H22, tmp_mat};
        H.push_back(H_vec3);
        vector<MatrixXd> H_vec4 = {tmp_mat, H31, tmp_mat, H33};
        H.push_back(H_vec4);
    }

    MatrixXd M_tmp, C_tmp, K_tmp, p_tmp;
    for (int i = 0; i < obj_num; i++) {
        M_tmp = readCSV(config.filepath + "M_" + to_string(i+1) + ".csv", N_equ[i], N_equ[i]);
        Mass.push_back(M_tmp);
        K_tmp = readCSV(config.filepath + "K_" + to_string(i+1) + ".csv", N_equ[i], N_equ[i]);
        K_tmp = K_tmp * config.YoungsModulus;
        K.push_back(K_tmp);
        C_tmp = config.coeff_alpha * M_tmp + config.coeff_beta * K_tmp;
        Damp.push_back(C_tmp);

        readCSV_ref(config.filepath + "p_0" + to_string(i+1)+".csv", N_equ[i], 1, p_tmp);
        p_0.push_back(p_tmp);


        MatrixXi tmp = readCSV_i(config.filepath + "f_surface_"+to_string(i+1)+".csv",
            config.element_size[i], 3);
        for (int j = 0; j < config.element_size[i]; j++) {
            for (int k = 0; k < 3; k++) {
                tmp(j, k) = tmp(j, k) - 1;
            }
        }

        f_surface.push_back(tmp);

    }
    mu = config.mu;
    cor = config.COR;
}

void Simulation::initMaleSimul() {
    cout << "init male simul" << endl;
    x_male.resize(3, N_simul);
    v_male.resize(6, N_simul);
    R_male.reserve(N_simul);

    M_male1 = MatrixXd::Identity(6, 6);

    m_male = config.m_male;
    I_male = config.I_male;

    x_male.col(0) = config.pnt_male;
    R_male[0] = Matrix3d::Identity();
    v_male.col(0) = VectorXd::Zero(6);

    male_com = config.male_com;

    if (config.control == 0) {
        M_male1.block<3, 3>(0, 0) = Matrix3d::Identity()/m_male;
        M_male1.block<3, 3>(3, 3) = Matrix3d::Identity()/I_male;
    } else if (config.control == 2) {
        M_im.resize(6, 6);
        B_im.resize(6, 6);
        K_im.resize(6, 6);
        M_im.setZero(); B_im.setZero(); K_im.setZero();
        M_im.block<3, 3>(0, 0) = 5 * Matrix3d::Identity();
        M_im.block<3, 3>(3, 3) = 5 * Matrix3d::Identity();
        K_im.block<3, 3>(0, 0) = 3000 * Matrix3d::Identity();
        K_im.block<3, 3>(3, 3) = 3000 * Matrix3d::Identity();

        for (int i = 0; i < 6; i++) {
            B_im(i, i) = 2*sqrt(M_im(i, i) * K_im(i, i));
        }
 
        readCSV_ref(config.filepath + "data20/16/pnt_traj.csv", 3, N_simul, pnt_traj);
        readCSV_ref(config.filepath + "data20/16/quat_traj.csv", 4, N_simul, quat_traj);

        M_male1 = (M_im/dt + B_im/2).inverse();
        M_male2 = -M_im/dt + B_im/2;

        e = VectorXd::Zero(6);
        e.head(3) = x_male.col(0) - pnt_traj.col(0);
        // quat: w, x, y, z
        Quaterniond quat(quat_traj(0, 0), quat_traj(1, 0), quat_traj(2, 0), quat_traj(3, 0));
        Matrix3d rot_traj(quat);
        Matrix3d rot_tmp = R_male[0] * rot_traj.transpose();
        AngleAxisd aa(rot_tmp);
        e.tail(3) = aa.angle() * aa.axis();
        Pvf_male = -M_male1 * K_im * e - M_male1*M_male2*v_male.col(0);

        R_male[0] = rot_traj;
        x_male.col(0) = pnt_traj.col(0);
    }
}

void Simulation::matrixConversionForVisualization(MatrixXf &m_rot, 
    MatrixXf &m_pos, vector<MatrixXf> &female_nodes) {
	vector<MatrixXf> node_x, node_y, node_z;
    for (int m = 0; m < obj_num; m++) {
        int total_node_num = floor(N_equ[m]/3.0) + node_const[m].size();
        node_x.push_back(MatrixXf::Zero(total_node_num, N_simul));
        node_y.push_back(MatrixXf::Zero(total_node_num, N_simul));
        node_z.push_back(MatrixXf::Zero(total_node_num, N_simul));

        MatrixXf one_vec = MatrixXf::Ones(1, N_simul);
        for (int i = 0; i < total_node_num; i++) {
            bool logi; int ind;
            ismember(i, node_const[m].transpose(), &logi, &ind);
            if (logi == true) {
                node_x[m].row(i) = one_vec * float(node_pnt_const[m](ind, 0));
                node_y[m].row(i) = one_vec * float(node_pnt_const[m](ind, 1));
                node_z[m].row(i) = one_vec * float(node_pnt_const[m](ind, 2));
            } else {
                int new_index = reduced_index(i, node_const[m].transpose());
                node_x[m].row(i) = x_global[m].row(3*new_index).cast<float>();
                node_y[m].row(i) = x_global[m].row(3*new_index + 1).cast<float>();
                node_z[m].row(i) = x_global[m].row(3*new_index + 2).cast<float>();
            }
        }
    }

    m_pos.resize(N_simul, 3);
    m_rot.resize(N_simul, 4);
    for (int i = 0; i < N_simul; i++) {
        Matrix3f rot = R_male[i].cast<float>();
        AngleAxisf aa(rot);
        m_rot(i, 0) = aa.axis()(0);
        m_rot(i, 1) = aa.axis()(1);
        m_rot(i, 2) = aa.axis()(2);
        m_rot(i, 3) = aa.angle();
        m_pos.row(i) = (R_male[i] * (-male_com) + male_com 
            + x_male.col(i)).transpose().cast<float>();
    }

    female_nodes = readFemaleData(node_x, node_y, node_z);
}

vector<MatrixXi> Simulation::get_f_surface() {
    return f_surface;
}

void Simulation::removeDuplicatedContacts(vector<Vector3i> &index,
    vector<Vector3d> &normal, vector<Vector3d> &contact_pnt,
    vector<double> &penet) {
    vector<Vector3i> new_index;
    vector<Vector3d> new_normal, new_contact_pnt;
    vector<double> new_penet;
    int num = 0;
    for (int i = 0; i < index.size(); i++) {
        bool logi; int ind;
        if (normal[i](1) >=0 && abs(normal[i].dot(Vector3d(0, 0, 1))) <= 0.8) {
            ismember_row(index[i], new_index, &logi, &ind);
            if (logi == false) {
                new_index.push_back(index[i]);
                new_normal.push_back(normal[i]);
                new_contact_pnt.push_back(contact_pnt[i]);
                new_penet.push_back(penet[i]);
            } else {
                if (penet[i] < new_penet[ind]) {
                    new_index[ind] = index[i];
                    new_normal[ind] = normal[i];
                    new_contact_pnt[ind] = contact_pnt[i];
                    new_penet[ind] = penet[i];
                }
            }
        }
    }
    index = new_index;
    normal = new_normal;
    contact_pnt = new_contact_pnt;
    penet = new_penet;
}

vector<MatrixXf> Simulation::readFemaleData(vector<MatrixXf> node_x, 
    vector<MatrixXf> node_y, vector<MatrixXf> node_z) {
	vector<MatrixXf> vec;
	for (int i = 0; i < obj_num; i++) {
		MatrixXf tmp(config.node_size[i]*N_simul, 3);
		for (int j = 0; j < N_simul; j++) {
			tmp.block(config.node_size[i]*j, 0, config.node_size[i], 1) = node_x[i].col(j);
			tmp.block(config.node_size[i]*j, 1, config.node_size[i], 1) = node_y[i].col(j);
			tmp.block(config.node_size[i]*j, 2, config.node_size[i], 1) = node_z[i].col(j);
		}
		vec.push_back(tmp);
	}
	return vec;
}

void Simulation::getContactPoints(MatrixXf &_contact_pnts, MatrixXf &_contact_normals) {
    _contact_pnts = contact_points.cast<float>();
    _contact_normals = contact_normals.cast<float>();
}