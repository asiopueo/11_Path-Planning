
#include "common.h"
#include <cmath>
#include <iostream>


double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}



// Coordinate transformations:
void global2local(std::vector<std::vector<double>> &trajectory, double angle, double t_x, double t_y)
{
	double tmp_x, tmp_y;

	for (int i=0; i<trajectory[0].size(); i++)
	{
		//cout << trajectory[0][i] << endl;
		tmp_x = (trajectory[0][i]-t_x)*cos(angle) + (trajectory[1][i]-t_y)*sin(angle);
		tmp_y = -(trajectory[0][i]-t_x)*sin(angle) + (trajectory[1][i]-t_y)*cos(angle);
			
		trajectory[0][i] = tmp_x;
		trajectory[1][i] = tmp_y;
    }
}

void local2global(std::vector<std::vector<double>> &trajectory, double angle, double t_x, double t_y)
{
	double tmp_x, tmp_y;

	for (int i=0; i<trajectory[0].size(); i++)
	{
		tmp_x = trajectory[0][i]*cos(angle) - trajectory[1][i]*sin(angle) + t_x;
		tmp_y = trajectory[0][i]*sin(angle) + trajectory[1][i]*cos(angle) + t_y;
		trajectory[0][i] = tmp_x;
		trajectory[1][i] = tmp_y;
	}
}




// Calculate the coefficients of a quintic polynomial from boundary coefficients
Eigen::VectorXd calculate_coefficients(Eigen::MatrixXd A_init, Eigen::VectorXd B_final)
{
	Eigen::MatrixXd Ai = A_init.inverse();
	Eigen::VectorXd coefficients = Ai * B_final;
	return coefficients;
}

// Evaluate quintic polynomial
double quintic_poly(Eigen::VectorXd coeffs, double x)
{
	double sum = 0;

	// Change the upper threshold!
	for (int i=0; i<6; i++)
		sum += coeffs(i) * pow(x,i);
	
	return sum;
}



