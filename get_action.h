#ifndef GET_ACTION_H
#define GET_ACTION_H

#include <eigen3/Eigen/Dense>

#include "Constants.h"
#include "rot_matrix_quat.h"
#include "lqr.h"

class get_action
{
public:
    get_action(){} // Call the System constructor.
    void get_action_one( float ch1,             // Input - Channel 1 [-1,1]
                     float ch2,             // Input - Channel 2 [-1,1]
                     float ch3,             // Input - Channel 3 [-1,1]
                     float ch4,             // Input - Channel 4 [-1,1]
                     float pos_x,           // Input - X-position measurement [m]
                     float pos_y,           // Input - Y-position measurement [m]
                     float pos_z,           // Input - Y-position measurement [m]
                     float vel_x,           // Input - X-velocity measurement [m/s]
                     float vel_y,           // Input - Y-velocity measurement [m/s]
                     float vel_z,           // Input - Z-velocity measurement [m/s]
                     float gyro_x,          // Input - X-gyroscope measurement [rad/s]
                     float gyro_y,          // Input - Y-gyroscope measurement [rad/s]
                     float gyro_z,          // Input - Z-gyroscope measurement [rad/s]
                     float q1,        	    // Input - quaternion 1 []
                     float q2,        	    // Input - quaternion 2 []
                     float q3,        	    // Input - quaternion 3 []
                     float q4,        	    // Input - quaternion 4 []
                     float& pwm_1_us,       // Output - Motor 1 PWM value [0,650] [us]
                     float& pwm_2_us,       // Output - Motor 2 PWM value [0,650] [us]
                     float& pwm_3_us,       // Output - Motor 3 PWM value [0,650] [us]
                     float& pwm_4_us);      // Output - Motor 4 PWM value [0,650] [us]

};

#endif /* GET_ACTION_H */
