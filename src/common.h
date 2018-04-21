#ifndef COMMON_H
#define COMMON_H


#include <vector>
using namespace std;

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
	unsigned int lane;
};

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

void global2vehicle(vector<vector<double>> &trajectory, pose ego_veh);
void vehicle2global(vector<vector<double>> &trajectory, pose ego_veh);



#endif /* COMMON_H */



