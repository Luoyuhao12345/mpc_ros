#include "mpc_util.h"

MpcUtil::MpcUtil()
{
	T = 0.1;
	Nx = 2;
	Nu = 1;
}

void MpcUtil::init_mat(float q, float r, int Np)
{
	horizon = Np;
    A << 1,0,
              0,1;
	B<<1,0;
	Phi.resize(Nx*horizon, Nx);
	Phi.setZero();
	Theta.resize(Nx*horizon, Nu*horizon);
	Theta.setZero();
	upperBound.resize(Nu*horizon);
	lowerBound.resize(Nu*horizon);
	_set_limit();
	A_I = Eigen::MatrixXd::Identity(horizon*Nu, horizon*Nu);
	_init_Q_R(q, r);
	_init_solver();
}

void MpcUtil::update_matrix(float vel)
{
	A(1,0) = T*vel;
	B(0,0) = T*vel/CAR_L;
	for (int i = 0; i < horizon; i++)
	{
		Phi.block(Nx*i, 0, Nx, Nx) = _pow_2d(A, i+1);
		for (int j = 0; j < horizon; j++)
		{
			if (i>=j)
			{
				Theta.block(Nx*i, Nu*j, Nx, Nu) = _pow_2d(A, i-j)*B;
			}
			else break;
		}
	}
}

float MpcUtil::get_result(float e_phi, float e_d, float ff, float car_x, float car_y, float car_yaw, float car_vel)
{
	Eigen::MatrixXd e;
	e.resize(2, 1);
	e << e_phi,e_d;
	Eigen::MatrixXd E = Phi*e;
	H = 2*(Theta.transpose()*Q*Theta+R);
	f = 2*Theta.transpose()*Q.transpose()*E;
	hessian = H.sparseView();
	gradient = f;
	linearMatrix = A_I.sparseView();
	solver.data()->setHessianMatrix(hessian);
	solver.data()->setGradient(gradient);
	solver.data()->setLinearConstraintsMatrix(linearMatrix);
	solver.data()->setLowerBound(lowerBound);
	solver.data()->setUpperBound(upperBound);
	solver.initSolver();
	solver.solveProblem();
	Eigen::VectorXd QPSolution;
	QPSolution = solver.getSolution();
	float steer = QPSolution[0]+ff;
	solver.data()->clearHessianMatrix();
	solver.data()->clearLinearConstraintsMatrix();
	solver.clearSolver();
	_traj_predict(QPSolution, ff, car_x, car_y, car_yaw, car_vel);
	return steer;
}

Eigen::Matrix2d MpcUtil::_pow_2d(Eigen::Matrix2d X, int n)
{
	Eigen::Matrix2d Y = Eigen::MatrixXd::Identity(Nx, Nx);
	for (int i = 0; i < n; i++)
	{
		Y = Y*X;
	}
	return Y;
}

void MpcUtil::_set_limit()
{
	int n = upperBound.rows();
	for (int i = 0; i < n; i++)
	{
		upperBound(i) = 1*pi/4;
		lowerBound(i) = -1*pi/4;
	}
}

void MpcUtil::_init_Q_R(float q, float r)
{
	Q = Eigen::MatrixXd::Identity(Nx*horizon, Nx*horizon)*q;
	R = Eigen::MatrixXd::Identity(Nu*horizon, Nu*horizon)*r;
}

void MpcUtil::_init_solver()
{
	solver.settings()->setWarmStart(true);
	solver.settings()->setVerbosity(false);
	solver.data()->setNumberOfVariables(Nu*horizon);
    solver.data()->setNumberOfConstraints(Nu*horizon);
}

void MpcUtil::_traj_predict(Eigen::VectorXd pred, float ff, float car_x, float car_y, float car_yaw, float car_vel)
{
	nav_msgs::Path pred_path;
	pred_path.header.frame_id = "map";
	// ros::Time current_time = ros::Time::now();
	float pred_yaw = car_yaw;
	float pred_x = car_x;
	float pred_y = car_y;
	int n = pred.rows();
	for(int i = 0; i<n; i++)
	{
		pred_yaw +=pred(i, 0);
		pred_x += T*car_vel*cos(pred_yaw);
		pred_y += T*car_vel*sin(pred_yaw);
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = pred_x;
		pose.pose.position.y = pred_y;
		pred_path.poses.push_back(pose);
	}
	traj_pred_pub.publish(pred_path);
}