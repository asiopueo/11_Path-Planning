#ifndef COMMON_H
#define COMMON_H


#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


struct pose {
	double pos_x;
	double pos_y;
	double angle;
	double s;
	double d;
};

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

void global2vehicle(std::vector<std::vector<double>>& trajectory, pose ego_veh);
void vehicle2global(std::vector<std::vector<double>>& trajectory, pose ego_veh);

Eigen::VectorXd calculate_coefficients(Eigen::MatrixXd init, Eigen::VectorXd final);


#endif /* COMMON_H */



