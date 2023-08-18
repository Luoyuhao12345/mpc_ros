#ifndef __MPC_UTIL_H_
#define __MPC_UTIL_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include </usr/local/include/eigen3/Eigen/Dense>
#include </usr/local/include/OsqpEigen/OsqpEigen.h>

#define CAR_L   0.26

using namespace std;
#define pi 3.14159

class MpcUtil 
{
public:
    MpcUtil();
    void update_matrix(float vel);
    float get_result(float e_phi, float e_d, float ff, float car_x, float car_y, float car_yaw, float car_vel);
    void init_mat(float q, float r, int Np);

private:
    Eigen::Matrix2d _pow_2d(Eigen::Matrix2d X, int n);
    void _set_limit();
    void _init_Q_R(float q, float r);
    void _init_solver();
    void _traj_predict(Eigen::VectorXd pred, float ff, float car_x, float car_y, float car_yaw, float car_vel);

public:
    ros::Publisher traj_pred_pub;

private:
    OsqpEigen::Solver solver;
    // 调用osqp-eigen的标准矩阵
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    // 方便自己使用的野矩阵
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Phi;
    Eigen::MatrixXd Theta;
    Eigen::MatrixXd A_I;
    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    int Nx, Nu;
    float T;
    int horizon;
};


#endif