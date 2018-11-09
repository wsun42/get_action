#ifndef ROT_MATRIX_QUAT_H
#define ROT_MATRIX_QUAT_H

#include <eigen3/Eigen/Dense>

#include "Constants.h"

void rot_matrix_quat(const Eigen::VectorXf&, Eigen::MatrixXf&);

#endif /* ROT_MATRIX_QUAT_H */