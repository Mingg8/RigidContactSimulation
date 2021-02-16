#include "utils.hh"


MatrixXd readCSV(string file, int rows, int cols) {
    ifstream in(file);
    string line;

    int row = 0;
    int col = 0;

    MatrixXd res(rows, cols);
    if (in.is_open()) {
        while (getline(in, line)) {
            char *ptr = (char *) line.c_str();
            int len = line.length();

            col = 0;

            char *start = ptr;
            for (int i = 0; i < len; i++) {
                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);
            row++;
        }
        in.close();
    }
    return res;
};

void readCSV_ref(string file, int rows, int cols, MatrixXd& res) {
    res.resize(rows, cols);
    ifstream in(file);
    string line;

    int row = 0;
    int col = 0;

    if (in.is_open()) {
        while (getline(in, line)) {
            char *ptr = (char *) line.c_str();
            int len = line.length();

            col = 0;

            char *start = ptr;
            for (int i = 0; i < len; i++) {
                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);
            row++;
        }
        in.close();
    }
};

MatrixXf readCSV_f(string file, int rows, int cols) {
    ifstream in(file);
    string line;

    int row = 0;
    int col = 0;

    MatrixXf res(rows, cols);
    if (in.is_open()) {
        while (getline(in, line)) {
            char *ptr = (char *) line.c_str();
            int len = line.length();

            col = 0;

            char *start = ptr;
            for (int i = 0; i < len; i++) {
                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);
            row++;
        }
        in.close();
    }
    return res;
};

MatrixXi readCSV_i(string file, int rows, int cols) {
    ifstream in(file);
    string line;

    int row = 0;
    int col = 0;

    MatrixXi res(rows, cols);
    if (in.is_open()) {
        while (getline(in, line)) {
            char *ptr = (char *) line.c_str();
            int len = line.length();

            col = 0;

            char *start = ptr;
            for (int i = 0; i < len; i++) {
                if (ptr[i] == ',') {
                    res(row, col++) = atoi(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atoi(start);
            row++;
        }
        in.close();
    }
    return res;
};

  
void ismember(int num, VectorXd vec, bool *logi, int *ind) {
    *logi = false;
    *ind = -1;
    for (int i = 0; i < vec.size(); i++) {
        if (num == floor(vec(i))) {
            *logi = true;
            *ind = i;
        }
    }
}

void ismember_row(Vector3i index, vector<Vector3i> vec, bool *logi, int *ind) {
    *logi = false;
    *ind = -1;
    for (int i = 0; i < vec.size(); i++) {
        if (index.isApprox(vec[i])) {
            *logi = true;
            *ind = i;
            break;
        }
    }
}

int reduced_index(int idx, VectorXd vec) {
    int new_pnt_idx = idx;
    for (int k = 0; k < vec.size(); k++) {
        if (idx > vec(k)) {
            new_pnt_idx--;
        } else {
            break;
        }
    }
    return new_pnt_idx;
}

VectorXd relu(VectorXd x) {
    VectorXd y(x.size());
    for (int i = 0; i < x.size(); i++) {
        if (x(i) >= 0) {
            y(i) = x(i);
        } else {
            y(i) = 0;
        }
    }
    return y;
}


MatrixXd pinv(MatrixXd mat) {
    MatrixXd pinv_mat;
    if (mat.rows() > mat.cols()) {
        pinv_mat = (mat.transpose() * mat).inverse() * mat.transpose();
    } else {
        pinv_mat = mat.transpose() * (mat * mat.transpose()).inverse();
    }
    return pinv_mat;
}

Matrix3d quat_to_rotmat(Vector4d q) {
    Matrix3d res = Matrix3d::Zero();
    res(0, 0) = pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2);
    res(0, 1) = 2*q(1)*q(2) - 2*q(0)*q(3);
    res(0, 2) = 2*q(0)*q(2) + 2*q(1)*q(3);
    res(1, 0) = 2*q(0)*q(2) + 2*q(1)*q(2);
    res(1, 1) = pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) - pow(q(3), 2);
    res(1, 2) = 2*q(2)*q(3) - 2*q(0)*q(1);
    res(2, 0) = 2*q(1)*q(3) - 2*q(0)*q(2);
    res(2, 1) = 2*q(2)*q(3) + 2*q(0)*q(1);
    res(2, 2) = pow(q(0), 2) - pow(q(1), 2) - pow(q(2), 2) + pow(q(3), 2);

    return res;
 }

 vector<int> unique(vector<Vector3i> vec) {
    vector<int> concat;
    for (int i = 0; i < vec.size(); i++) {
        for (int j = 0; j < 3; j++) {
            concat.push_back(vec[i](j));
        }        
    }
    sort(concat.begin(), concat.end());
    auto last = unique(concat.begin(), concat.end());
    concat.erase(last, concat.end());
    return concat;
 }


Matrix3d Skew(Vector3d vec) {
    Matrix3d res;
    res << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return res;
}

Matrix3d w_to_rot(Vector3d w) {
    Matrix3d R;
    double theta = w.norm();
    double eps = 1e-5;
    if (theta < eps) {
        R = Matrix3d::Identity();
    } else {
        w = w / theta;
        Matrix3d S;
        S << 0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
        R = Matrix3d::Identity() + S * sin(theta) + S * S * (1-cos(theta));
    }
    return R;
}

VectorXd tanh(VectorXd a) {
    for (int i = 0; i < a.size(); i++) {
        a(i) = tanh(a(i));
    }
    return a;
}

// template <typename T>
vector<int> sort_indexes(const vector<double> &v, int num) {
    vector<int> idx(v.size());
    iota(idx.begin(), idx.end(), 0);
    stable_sort(idx.begin(), idx.end(),
        [&v](int i1, int i2){return v[i1] < v[i2];});

    vector<int> res(idx.begin(), idx.begin() + num);
    return res;
}

MatrixXd jaco_franka(VectorXd q) {
    vector<MatrixXd> Jaco;
    vector<MatrixXd> SE3;
    VectorXd a, d, alpha;
    a.resize(8); d.resize(8); alpha.resize(8);
    a <<     0,     0,     0, 0.0825, -0.0825,    0, 0.088, 0;
    d << 0.33,     0, 0.316,      0,   0.384,    0,     0, 0.107;
    alpha << 0, -PI/2,  PI/2,   PI/2,   -PI/2, PI/2,  PI/2, 0;

    SE3.push_back(mod_DH_T(a(0), d(0), alpha(0), q(0)));
    for (int i = 0; i < 6; i++) {
        SE3.push_back(SE3[i] *mod_DH_T(a(i+1), d(i+1), alpha(i+1), q(i+1)));
    }
    SE3.push_back(SE3[6] * mod_DH_T(a(7), d(7), alpha(7), 0));

    MatrixXd J_to_link(6, 7);
    for (int col = 0; col < 7; col++) {
        Vector3d tmp = SE3[col].block<3, 1>(0, 2);
        J_to_link.block<3, 1>(0, col) = tmp.cross(SE3[7].block<3, 1>(0, 3) 
            - SE3[col].block<3,1>(0, 3));
        J_to_link.block<3, 1>(3, col) = SE3[col].block<3, 1>(0, 2);
    }
    return J_to_link;
}

VectorXd ik_franka(MatrixXd T_des, VectorXd q_ini, int max_loop_iter) {
    VectorXd q;
    q.resize(7);
    VectorXd q_lim_u, q_lim_l;
    q_lim_u.resize(7); q_lim_l.resize(7);
    q_lim_u << 166, 101, 166, -4, 166, 215, 166;
    q_lim_l << -166, -101, -166, -176, -166, -1, -166;
    q_lim_u = q_lim_u * PI/180;
    q_lim_l = q_lim_l * PI/180;

    VectorXd q_mid = (q_lim_l + q_lim_u) / 2.0;
    VectorXd q_range = q_lim_u - q_mid;
    MatrixXd weight(7, 7);
    weight.setZero();
    weight(0, 0) = 1.0; weight(1, 1) = 0.5; weight(2, 2) = 0.1;
    weight(3, 3) = 0.2; weight(4, 4) = 0.1; weight(5, 5) = 0.2;
    weight(6, 6) = 0.1;

    if (q_ini.norm() == 0) {
        VectorXd q_tmp; q_tmp.resize(7);
        q_tmp << 0, 0, 0, 0.1, 0.5, 0, 0;
        q = q_mid + q_tmp;
    } else {
        q = q_ini;
    }
    VectorXd q_old = q;
    int loop_iter = 0;
    while (true) {
        loop_iter++;
        vector<MatrixXd> T = fk_franka(q);
        Vector3d p = T[7].block<3, 1>(0, 3);
        Matrix3d R = T[7].block<3, 3>(0, 0);
        MatrixXd J = jaco_franka(q);
        MatrixXd jaco_p = J.block<3, 7>(0, 0);
        Vector3d p_des = T_des.block<3, 1>(0, 3);
        Matrix3d R_des = T_des.block<3, 3>(0, 0);

        MatrixXd J_dagger = weight * J.transpose() * (J*weight*J.transpose() + 0.00001 * MatrixXd::Identity(6, 6)).inverse();

        VectorXd error = VectorXd::Zero(6);
        error.head(3) = p - p_des;
        AngleAxisd aa(R * R_des.transpose());
        error.tail(3) = aa.axis() * aa.angle();
        VectorXd del_q = -J_dagger * error + (MatrixXd::Identity(7, 7) - J_dagger * J)
            * weight * (q_mid - q);
        q = q + del_q * 0.2;

        if (error.norm() < 0.001 || loop_iter > max_loop_iter) {
            break;
        }
    }
    return q;
}

vector<MatrixXd> fk_franka(VectorXd q) {
    VectorXd a, d, alpha;
    a.resize(8); d.resize(8); alpha.resize(8);
    a <<     0,     0,     0, 0.0825, -0.0825,    0, 0.088, 0;
    d << 0.33,     0, 0.316,      0,   0.384,    0,     0, 0.107;
    alpha << 0, -PI/2,  PI/2,   PI/2,   -PI/2, PI/2,  PI/2, 0;

    MatrixXd SE3 = MatrixXd::Identity(4, 4);
    vector<MatrixXd> T_out;
    for (int i = 0; i < 7; i++) {
        SE3 = SE3 * mod_DH_T(a(i), d(i), alpha(i), q(i));
        T_out.push_back(SE3);
    }
    SE3 = SE3 * mod_DH_T(a(7), d(7), alpha(7), 0);
    T_out.push_back(SE3);
    return T_out;
}

MatrixXd mod_DH_T(double a, double d, double alpha, double theta) {
    MatrixXd SE3;
    SE3 = SE3_gen(w_to_rot(alpha * Vector3d(1, 0, 0)), Vector3d(0, 0, 0))
        * SE3_gen(Matrix3d::Identity(), Vector3d(a, 0, 0))
        * SE3_gen(w_to_rot(theta * Vector3d(0, 0, 1)), Vector3d(0, 0, 0))
        * SE3_gen(Matrix3d::Identity(), Vector3d(0, 0, d));
    return SE3;
}

MatrixXd SE3_gen(Matrix3d R, Vector3d p) {
    MatrixXd SE3 = MatrixXd::Identity(4, 4);
    SE3.block<3, 3>(0, 0) = R;
    SE3.block<3, 1>(0, 3) = p;
    return SE3;
}