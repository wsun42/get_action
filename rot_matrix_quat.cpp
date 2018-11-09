
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

#include "rot_matrix_quat.h"

using namespace Eigen;
using namespace std;
using namespace Constants;

/**
 rot_matrix_quat obtains the rotation matrix R from the quaternion input q
 @param q -  quaternion
 @return R - rotation matrix wrt q
 */
void rot_matrix_quat(const VectorXf& q, MatrixXf& R){
    VectorXf tmp = q;

    // Normalization
    float a = q(0) / tmp.norm();
    float b = q(1) / tmp.norm();
    float c = q(2) / tmp.norm();
    float d = q(3) / tmp.norm();

    MatrixXf Rt(3,3);
    Rt.setZero();
    
    Rt(0,0) = pow(a,2) + pow(b,2) - pow(c,2) - pow(d,2);
    Rt(0,1) = 2*(b*c - a*d);
    Rt(0,2) = 2*(a*c + b*d);
    Rt(1,0) = 2*(b*c + a*d);
    Rt(1,1) = pow(a,2) - pow(b,2) + pow(c,2) - pow(d,2);
    Rt(1,2) = 2*(c*d - a*b);
    Rt(2,0) = 2*(b*d - a*c);
    Rt(2,1) = 2*(a*b + c*d);
    Rt(2,2) = pow(a,2) - pow(b,2) - pow(c,2) + pow(d,2);
    
    R=Rt;

}