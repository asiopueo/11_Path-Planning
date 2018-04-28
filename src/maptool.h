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
		std::vector<double> parabolicGetXY(double s, double d);

	private:
		std::vector<double> map_waypoints_x;
		std::vector<double> map_waypoints_y;
		std::vector<double> map_waypoints_s;
		std::vector<double> map_waypoints_dx;
		std::vector<double> map_waypoints_dy;

		std::vector<double> parabolicInterpol(std::vector<double> X, std::vector<double> Y, int center, double ds, double d);
		double calcPoly(std::vector<double> coeffs, double t);

		int ClosestWaypoint(double x, double y);
		int NextWaypoint(double x, double y, double theta);
};



#endif /* MAPTOOL_H */