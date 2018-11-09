#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <fstream>
#include <cstdio>
#include <ctime>

#include "get_action.h"

using namespace std;
using namespace Constants;
using namespace Eigen;

void print_trajectory(vector<VectorXd> traj)
{
    std::cout<< "trajectory " <<endl;
    for(int i=0; i<num_time_steps-1 ;i=i+5) {
        std::cout << traj[i].transpose() <<endl;
    }
    
} //print_traj


/**
    This function will run the Differential Dynamic Programming trajectory
    optimization algorithm
*/
int main()
{
    int start_s=clock();
    /*  BEGIN INITIAL AND TARGET STATE INITIALIZATION
        x_0 is num_states x 1 vector of initial states
        x_target is num_states x 1 vector of target states
    */
    VectorXd x_0(20);       // do not edit
    x_0.setZero();                 // initial state using comma initialization
//    x_0 << 0,0,pi,0;
    x_0(0) = 1;

    float pwm_1, pwm_2, pwm_3, pwm_4;
    get_action get_u;
    get_u.get_action_one(0,0,0,0,0,0,0,0,0,0,0,0,0,0.25,0.25,0.25,0.25,pwm_1,pwm_2,pwm_3,pwm_4);

    int stop_s=clock();
    printf("Entire Execution Time: %li ms\n",1000*(stop_s-start_s)/CLOCKS_PER_SEC);

    return 0;
}
