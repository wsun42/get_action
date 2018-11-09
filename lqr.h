#ifndef LQR_H
#define LQR_H

#include <eigen3/Eigen/Dense>


struct MatrixPair { 
	Eigen::MatrixXf U;
	Eigen::MatrixXf S;
};


Eigen::MatrixXf lqr(Eigen::MatrixXf& A, Eigen::MatrixXf& B, Eigen::MatrixXf& Q_x, Eigen::MatrixXf& R_u);

#endif