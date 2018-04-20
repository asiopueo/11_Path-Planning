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
enum state {LK, PLCL, PLCR, LCL, LCR};

struct pose {
	double pos_x;
	double pos_y;
	double angle;
	double s;
	double d;
	unsigned int lane;
};

void global2vehicle(vector<vector<double>> &trajectory, pose ego_veh);
void global2vehicle(vector<vector<double>> &trajectory, pose ego_veh);



#endif /* COMMON_H */



