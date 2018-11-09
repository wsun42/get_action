#include "Constants.h"

namespace Constants {

    /*
        IMPORTANT - do not name any constants with name:
        l, L, g, G, q_0, q_u, q_uu, q_ux, q_x, q_xx
        R, Q_x, Q_f, A, B, x_0, x_target, cost, V, V_x, V_xx
    */

    // Universal Constant ints
    // You can add, but do not change
    extern const float pi(std::atan(1)*4);
    extern const float grav(9.81);

    // DDP Constant ints
    // Change as necessary, do not remove
//    
   //Quadrotor system
    extern const int num_states(13);
    extern const int num_controls_u(4);
//    
    extern const int num_time_steps(101);
    // extern const int num_iterations(20);

    // DDP Constant doubles
    // Change as necessary, do not remove
    extern const float dt(0.01);
    extern const float learning_rate(0.3);

    // System Constant doubles
    // Change, add, or remove as necessary
    extern const float m1(0.33);
    extern const float M1(10.0);
    extern const float length1(0.085);
    extern const float Ixx(0.000781716);
    extern const float Iyy(0.000937379);
    extern const float Izz(0.001581030);
    // extern const float f_max(3.8);
    // extern const float f_min(0.2);
    extern const float f_max(2.2217);
    extern const float f_min(0.0900);
    extern const float gamma1(0.00275);
    extern const float k_tau(0.0121);
    extern const float k_F(0.0000013655);

    // K gains
    extern const float Kp1(2.25);
    extern const float Kp2(2.25);
    extern const float Kp3(9.0);
    extern const float Kd1(2.1);
    extern const float Kd2(2.1);
    extern const float Kd3(6.0);

    // final position
    extern const float rf1(0.0);
    extern const float rf2(0.0);
    extern const float rf3(1.5);

    // primal axis of rotation at equilibrium in the body frame
    extern const float n_x_eq(-0.37676166354854);
    extern const float n_y_eq(0.467879133781298);
    extern const float n_z_eq(0.799462172371113);  

    // primal axis of rotation at equilibrium in the body frame
    extern const float p_eq(-6.37078073389549);
    extern const float q_eq(7.911514518786770);
    extern const float r_eq(13.51835576169825);  
    
    // thrust input at equilibrium 
    extern const float T1_eq(1.619738925431959);
    extern const float T2_eq(0.809869462715980);
    extern const float T3_eq(1.619738925431959);   
    extern const float T4_eq(0.0);   

    // coefficients for omega to PWM transformation
    extern const float p00(1316.0);
    extern const float p10(-71.42);
    extern const float p01(1.145);   
    extern const float p20(3.198);   
    extern const float p11(-0.05007);   
    extern const float p02(0.0001435);   

}
