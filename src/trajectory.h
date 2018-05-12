#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "common.h"
#include "maptool.h"


class Trajectory
{
	public:
		Trajectory(pose, double delta_s, double delta_d, double current_speed, double target_speed, double time);
		~Trajectory();

		trajectory_t getXY(Maptool map);
		//double getFinal_s();
		//double getFinal_d();
		double evaluate_s(double t);
		double evaluate_d(double t);

	private:
		Eigen::VectorXd coeffs_s;
		Eigen::VectorXd coeffs_d;

		double T;

};




#endif