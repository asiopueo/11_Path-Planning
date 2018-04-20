
#include "common.h"
#include <cmath>

// Coordinate transformations:
void global2vehicle(vector<vector<double>> &trajectory, pose ego_veh)
{
	double tmp_x, tmp_y;

	for (int i=0; i<trajectory.size(); i++)
	{
		tmp_x = (trajectory[0][i]-ego_veh.pos_x)*cos(ego_veh.angle)-(trajectory[1][i]-ego_veh.pos_y)*sin(ego_veh.angle);
		tmp_y = (trajectory[0][i]-ego_veh.pos_x)*sin(ego_veh.angle)+(trajectory[1][i]-ego_veh.pos_y)*cos(ego_veh.angle);
			
		trajectory[0][i] = tmp_x;
		trajectory[1][i] = tmp_y;
    }
}

void vehicle2global(vector<vector<double>> &trajectory, pose ego_veh)
{
	double tmp_x, tmp_y;

	for (int i; i<trajectory.size(); i++)
	{
		tmp_x = trajectory[0][i]*cos(ego_veh.angle)+trajectory[1][i]*sin(ego_veh.angle) + ego_veh.pos_x;
		tmp_y = trajectory[0][i]*sin(ego_veh.angle)+trajectory[1][i]*cos(ego_veh.angle) + ego_veh.pos_y;

		trajectory[0][i] = tmp_x;
		trajectory[1][i] = tmp_y;
	}
}