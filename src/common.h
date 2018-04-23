#ifndef COMMON_H
#define COMMON_H


#include <vector>
using namespace std;



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

void global2vehicle(vector<vector<double>>& trajectory, pose ego_veh);
void vehicle2global(vector<vector<double>>& trajectory, pose ego_veh);



#endif /* COMMON_H */



