#include "trajectory.h"
#include <iostream>

Trajectory::Trajectory(pose egoPose, double delta_s, double delta_d, double current_speed, double target_speed, double time_T)
{
	// Check traffic
	T = time_T; // Total time for trajectory [sec]
	unsigned int number_of_steps;

	// Calculation for s(t):
	Eigen::MatrixXd matrix_s = Eigen::MatrixXd(3,3);
	Eigen::VectorXd vector_s = Eigen::VectorXd(3);
	Eigen::VectorXd s_i = Eigen::VectorXd(3);
	Eigen::VectorXd s_f = Eigen::VectorXd(3);

	matrix_s << pow(T,3), pow(T,4), pow(T,5),
				3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
				6*pow(T,1), 12*pow(T,2), 20*pow(T,3);

	s_i << egoPose.s, current_speed, 0.;	// 50mph = 22.352 m/s
	s_f << egoPose.s + delta_s, target_speed, 0.;

	vector_s << s_f(0) - (s_i(0)+s_i(1)*T+0.5*s_i(2)*pow(T,2)), 
				s_f(1) - (s_i(1)+s_i(2)*T), 
				s_f(2) -  s_i(2);

	// Calculation for d(t):
	Eigen::MatrixXd matrix_d = Eigen::MatrixXd(3,3);
	Eigen::VectorXd vector_d = Eigen::VectorXd(3);
	Eigen::VectorXd d_i = Eigen::VectorXd(3);
	Eigen::VectorXd d_f = Eigen::VectorXd(3);

	matrix_d << pow(T,3), pow(T,4), pow(T,5),
				3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
				6*pow(T,1), 12*pow(T,2), 20*pow(T,3);

	d_i << egoPose.d, 0., 0.;
	d_f << egoPose.d + delta_d, 0., 0.;

	vector_d << d_f(0) - (d_i(0)+d_i(1)*T+0.5*d_i(2)*pow(T,2)), 
				d_f(1) - (d_i(1)+d_i(2)*T), 
				d_f(2) -  d_i(2);


	// Result of these calculations are the coefficients \alpha_3, \alpha_4, and \alpha_5
	Eigen::VectorXd result_s = calculate_coefficients(matrix_s, vector_s);
	Eigen::VectorXd result_d = calculate_coefficients(matrix_d, vector_d);

	coeffs_s = Eigen::VectorXd(6);
	coeffs_d = Eigen::VectorXd(6);

	coeffs_s << s_i(0), s_i(1), 0.5*s_i(2), result_s(0), result_s(1), result_s(2);
	coeffs_d << d_i(0), d_i(1), 0.5*d_i(2), result_d(0), result_d(1), result_d(2);

	/*for (int i=0; i<6; i++)
		std::cout << coeffs_s(i) << "\t";
	std::cout << std::endl;

	for (int i=0; i<6; i++)
		std::cout << coeffs_d(i) << "\t";
	std::cout << std::endl;*/
}


Trajectory::~Trajectory(){}

// Evaluate quintic polynomial
double Trajectory::evaluate_s(double t)
{
	double sum = 0;
	for (int i=0; i<6; i++)
		sum += coeffs_s(i) * pow(t,i);
	
	return sum;
}

double Trajectory::evaluate_d(double t)
{
	double sum = 0;
	for (int i=0; i<6 ; i++)
		sum += coeffs_d(i) * pow(t,i);
	
	return sum;
}

/*double Trajectory::getFinal_s()
{
	
	return s;
}

double Trajectory::getFinal_d()
{
	return d;
}*/



trajectory_t Trajectory::getXY(Maptool map)
{
	trajectory_t trajectory(2);

	// Prefactor of 50 accounts for 50Hz/20ms update frequency
	int number_of_steps = T / 0.02;

	// In order to eliminate jiggering in longitudinal direction.
	for(int i=0; i < number_of_steps+1 ; i++)
	{		      
		trajectory[0].push_back(evaluate_s(0.02*i));
		trajectory[1].push_back(evaluate_d(0.02*i));
	}

	//std::cout << trajectory[0].size() << std::endl;
	
	for (int i=0; i<trajectory[0].size(); i++) 
	{
		std::vector<double> tmpXY;
		//tmpXY = map.getXY(trajectory[0][i], trajectory[1][i]);
		tmpXY = map.getXY_spline(trajectory[0][i], trajectory[1][i]);
		trajectory[0][i] = tmpXY[0];
		trajectory[1][i] = tmpXY[1];
	}

	return trajectory;
}





