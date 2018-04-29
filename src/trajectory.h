#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "common.h"
#include "maptool.h"


class Trajectory
{
	public:
		Trajectory(pose, double delta_s, double delta_d);
		~Trajectory();

		trajectory_t getXY(Maptool map);

	private:
		Eigen::VectorXd coeffs_s;
		Eigen::VectorXd coeffs_d;

		double evaluate_s(double t);
		double evaluate_d(double t);
};




#endif