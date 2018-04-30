#ifndef COMMON_H
#define COMMON_H


#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"



/*
 * STAY:  Stay on current lane
 * PLCL:  Prepare Lane Change Left
 * PLCR:  Prepare Lane Change Right
 * LCL:   Lane Change Left
 * LCR:   Lane Change Right
 */

enum state {LK, PLCL, PLCR, LCL, LCR, EA};

struct pose {
	double pos_x;
	double pos_y;
	double angle;
	double s;
	double d;
};

typedef std::vector<std::vector<double>> trajectory_t;
typedef std::vector<std::vector<double>> targetList_t;

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

void global2local(trajectory_t& trajectory, double angle, double t_x, double t_y);
void local2global(trajectory_t& trajectory, double angle, double t_x, double t_y);

Eigen::VectorXd calculate_coefficients(Eigen::MatrixXd init, Eigen::VectorXd final);

double quintic_poly(Eigen::VectorXd coeffs, double x);

#endif /* COMMON_H */



