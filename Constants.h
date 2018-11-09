#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

namespace Constants {

    extern const float pi;
    extern const float grav;

    extern const int num_states;
    extern const int num_controls_u;
	extern const int num_controls_v;
    extern const int num_time_steps;
    extern const int num_iterations;

    extern const float dt;
    extern const float learning_rate;

    extern const float m1;
    extern const float M1;
    extern const float length1;
    
    extern const float Ixx;
    extern const float Iyy;
    extern const float Izz;
    
    extern const float f_max;
    extern const float f_min;
    extern const float gamma1;
    extern const float k_tau;
    extern const float k_F;
    
    extern const float Kp1;
    extern const float Kp2;
    extern const float Kp3;
    extern const float Kd1;
    extern const float Kd2;
    extern const float Kd3;
    extern const float rf1;
    extern const float rf2;
    extern const float rf3;
    extern const float n_x_eq;
    extern const float n_y_eq;
    extern const float n_z_eq;
    extern const float p_eq;
    extern const float q_eq;
    extern const float r_eq;
    extern const float T1_eq;
    extern const float T2_eq;
    extern const float T3_eq;
    extern const float T4_eq;

    extern const float p00;
    extern const float p10;
    extern const float p01;
    extern const float p20;
    extern const float p11;
    extern const float p02;

}

#endif // CONSTANTS_H
