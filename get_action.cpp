
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>
#include <cmath>

#include "get_action.h"

using namespace Eigen;
using namespace std;
using namespace Constants;

// Start your modifications here.
  // Fill in this part... (if necessary)
// End your modifications here.

void get_action::get_action_one( float ch1,
                     float ch2,
                     float ch3,
                     float ch4,
                     float pos_x,
                     float pos_y,
                     float pos_z,
                     float vel_x,
                     float vel_y,
                     float vel_z,
                     float gyro_x,
                     float gyro_y,
                     float gyro_z,
                     float q1,
                     float q2,
                     float q3,
                     float q4,
                     float& pwm_1_us,
                     float& pwm_2_us,
                     float& pwm_3_us,
                     float& pwm_4_us) {
  // Start your modifications here.

  // Initialization
    VectorXf r(3), v(3), q(4), w(3), rf(3), n_g(3); // pos, vel, quaternion, ang vel
    MatrixXf R(3,3), Kd(3,3), Kp(3,3); // rotation matrix
    r << pos_x, pos_y, pos_z;
    v << vel_x, vel_y, vel_z;
    q << q1, q2, q3, q4;
    w << gyro_x, gyro_y, gyro_z;
    rf << rf1, rf2, rf3;
    n_g << 0, 0, -grav;
    R.setZero();
    Kd.setZero();
    Kp.setZero();
    rot_matrix_quat(q, R);
    Kd(0,0) = Kd1;
    Kd(1,1) = Kd2;
    Kd(2,2) = Kd3;
    Kp(0,0) = Kp1;
    Kp(1,1) = Kp2;
    Kp(2,2) = Kp3;

    cout << "R = \n" << R << endl;

// find desired primal rotation vector n_des
    VectorXf d2dot_des = -Kd*v - Kp*(r - rf);
    VectorXf tmp = m1/n_z_eq*R.transpose()*(d2dot_des - n_g);
    float T_sigma = tmp.norm();
    VectorXf n_des = tmp/tmp.norm();

// find rotation matrix from body frame B to control coordinate frame C
    MatrixXf C_c2b(3,3);
    C_c2b.setZero();
    C_c2b.col(2) << n_des(0), n_des(1), n_des(2);
    Vector3f tmp1, tmp2, tmp3;
    tmp1 << 0, 1, 0;
    tmp2 << n_des(0), n_des(1), n_des(2);
    tmp3 = tmp1.cross(tmp2);
    if (tmp3.norm() == 0) {
      tmp1 << 1, 0, 0;
      tmp3 = tmp1.cross(tmp2);
    }
    tmp3 = tmp3/tmp3.norm();
    C_c2b.col(1) << tmp3(0), tmp3(1), tmp3(2);
    tmp1 = tmp3.cross(tmp2);
    tmp1 = tmp1/tmp1.norm();
    C_c2b.col(0) << tmp1(0), tmp1(1), tmp1(2);
    C_c2b.transposeInPlace();
    VectorXf w_c = C_c2b*w;

    cout << "C_c2b = \n" << C_c2b << endl;

    // RealSchur<MatrixXf> schur(C_c2b);

    MatrixXf Ao(3,3);
    Ao << 0, (Iyy-Izz)*r_eq/Ixx, (Iyy-Izz)*q_eq/Ixx,
          (Izz-Ixx)*r_eq/Iyy, 0, (Izz-Ixx)*p_eq/Iyy,
          0, 0, -gamma1/Izz;
    MatrixXf Ac = C_c2b*Ao*C_c2b.transpose();
    MatrixXf Btmp(2,2);
    Btmp << length1/Ixx, 0, 
            0, length1/Iyy;
    MatrixXf Bc = C_c2b.block(0,0,2,2)*Btmp;

    MatrixXf A(4,4);
    A.setZero();
    A.block(0,0,2,2) << Ac(0,0), Ac(0,1),
                        Ac(1,0), Ac(1,1);
    VectorXf w_eq(3);
    w_eq << p_eq, q_eq, r_eq;
    A.block(2,0,2,4) << 0, -1, 0, w_eq.norm(),
                        1, 0, -w_eq.norm(), 0;
    MatrixXf B(4,2);
    B.setZero();
    B.block(0,0,2,2) << Bc(0,0), Bc(0,1),
                        Bc(1,0), Bc(1,1);

    MatrixXf Q_x(4,4);
    Q_x << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 15, 0,
         0, 0, 0, 15;

    MatrixXf R_u(2,2);
    R_u << 20, 0, 
           0, 20;

    // cout << "Ao = \n" << Ao << endl;
    // cout << "Ac = \n" << Ac << endl;
    // cout << "Bc = \n" << Bc << endl;
    cout << "A = \n" << A << endl;
    cout << "B = \n" << B << endl;
    cout << "Q_x = \n" << Q_x << endl;
    cout << "R_u = \n" << R_u << endl;

    // Find lqr gain of linearized dynamics
    MatrixXf K(2,4);
    K = lqr(A, B, Q_x, R_u);

    cout << "K = \n" << K << endl;


    VectorXf w_c_n(4);
    w_c_n << w_c(0), w_c(1), 0, 0;
    VectorXf u_lin(2); // control of linearized sys
    u_lin = - K*w_c_n;
    // u_lin << 0.1, 0.2;

    // calculate thrust inputs from Tm*u_ref = bm
    // T4 = 0, loss of rotor no. 4
    MatrixXf Tm_inv(3,3); // inverse of Tm
    Tm_inv << 0.0, -0.5, 0.5,
          0.5, 0.5, 0.0,
          -0.5, 0.0, 0.5;
    VectorXf bm(3);
    bm(0) = u_lin(0) + T1_eq + T2_eq - T3_eq - T4_eq;
    bm(1) = u_lin(1) - T1_eq + T2_eq + T3_eq - T4_eq;
    bm(2) = T_sigma;
    VectorXf u_ref = Tm_inv * bm;
    // cout << "u_ref_ori = \n" << u_ref << endl;

    // satsify constrait by concatenation
    for (int i = 0; i<3; i++) {
      if (u_ref(i) < f_min) {
        u_ref(i) = f_min;
      } 
      else if (u_ref(i) > f_max) {
        u_ref(i) = f_max;
      }
    } // for i = 0

    // cout << "bm = \n" << bm << endl;
    // cout << "u_ref = \n" << u_ref << endl;

    VectorXf omega_ref(4);
    omega_ref(0) = sqrt(u_ref(0)/k_F);
    omega_ref(1) = sqrt(u_ref(1)/k_F);
    omega_ref(2) = sqrt(u_ref(2)/k_F);
    omega_ref(3) = 0;

    float x = 12.0; // voltage at almost full level
    float y = omega_ref(0);
    float pwm_1 = p00 + p10*x + p01*y + p20*pow(x,2) + p11*x*y + p02*pow(y,2);
    y = omega_ref(1);
    float pwm_2 = p00 + p10*x + p01*y + p20*pow(x,2) + p11*x*y + p02*pow(y,2);
    y = omega_ref(2);
    float pwm_3 = p00 + p10*x + p01*y + p20*pow(x,2) + p11*x*y + p02*pow(y,2);
    y = omega_ref(3);
    float pwm_4 = p00 + p10*x + p01*y + p20*pow(x,2) + p11*x*y + p02*pow(y,2);

    pwm_1_us = pwm_1;
    pwm_2_us = pwm_2;
    pwm_3_us = pwm_3;
    pwm_4_us = pwm_4;

    cout << "pwm_1 = \n" << pwm_1_us << endl;
    cout << "pwm_2 = \n" << pwm_2_us << endl;
    cout << "pwm_3 = \n" << pwm_3_us << endl;
    cout << "pwm_4 = \n" << pwm_4_us << endl;

  // End your modifications here.
} 



       