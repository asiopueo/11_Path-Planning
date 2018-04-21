#ifndef MAPTOOL_H
#define MAPTOOL_H

#include <vector>


class Maptool
{
	public:
		Maptool();
		~Maptool();

		std::vector<double> getFrenet(double x, double y, double theta);
		std::vector<double> getXY(double s, double d);


	private:
		std::vector<double> map_waypoints_x;
		std::vector<double> map_waypoints_y;
		std::vector<double> map_waypoints_s;
		std::vector<double> map_waypoints_dx;
		std::vector<double> map_waypoints_dy;

		int ClosestWaypoint(double x, double y);
		int NextWaypoint(double x, double y, double theta);
};



#endif /* MAPTOOL_H */