
#include "common.h"
#include <math.h>
#include <iostream>


double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}



// Coordinate transformations:
void global2vehicle(std::vector<std::vector<double>> &trajectory, pose ego_veh)
{
	double tmp_x, tmp_y;

	for (int i=0; i<trajectory[0].size(); i++)
	{
		//cout << trajectory[0][i] << endl;
		tmp_x = (trajectory[0][i]-ego_veh.pos_x)*cos(ego_veh.angle) + (trajectory[1][i]-ego_veh.pos_y)*sin(ego_veh.angle);
		tmp_y = -(trajectory[0][i]-ego_veh.pos_x)*sin(ego_veh.angle) + (trajectory[1][i]-ego_veh.pos_y)*cos(ego_veh.angle);
			
		trajectory[0][i] = tmp_x;
		trajectory[1][i] = tmp_y;
    }
}

void vehicle2global(std::vector<std::vector<double>> &trajectory, pose ego_veh)
{
	double tmp_x, tmp_y;

	for (int i=0; i<trajectory[0].size(); i++)
	{
		tmp_x = trajectory[0][i]*cos(ego_veh.angle) - trajectory[1][i]*sin(ego_veh.angle) + ego_veh.pos_x;
		tmp_y = trajectory[0][i]*sin(ego_veh.angle) + trajectory[1][i]*cos(ego_veh.angle) + ego_veh.pos_y;
		trajectory[0][i] = tmp_x;
		trajectory[1][i] = tmp_y;
	}
}




// Calculate the coefficients of a quintic polynomial from boundary coefficients
Eigen::VectorXd calculate_coefficients(Eigen::MatrixXd A_init, Eigen::VectorXd B_final)
{
	Eigen::MatrixXd Ai = A_init.inverse();
	Eigen::VectorXd coefficients = Ai*B_final;
	return coefficients;
}

// Evaluate quintic polynomial
double quintic_poly(Eigen::VectorXd coeffs, double x)
{
	double result = 0;

	// Change the upper threshold!
	for (int i=0; i<3; i++)
		result += coeffs(i) * pow(x,i);
	
	return result;
}



