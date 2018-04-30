#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include "maptool.h"
#include <iostream>
#include "spline.h"
#include "common.h"

Maptool::Maptool()
{
	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	// Waypoint map to read from
	std::string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	std::string line;
	while (getline(in_map_, line)) 
	{
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
}


Maptool::~Maptool() {
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> Maptool::getFrenet(double x, double y, double theta)
{
	int next_wp = NextWaypoint(x,y, theta);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = map_waypoints_x.size()-1;
	}

	double n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
	double n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];
	double x_x = x - map_waypoints_x[prev_wp];
	double x_y = y - map_waypoints_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-map_waypoints_x[prev_wp];
	double center_y = 2000-map_waypoints_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(map_waypoints_x[i],map_waypoints_y[i],map_waypoints_x[i+1],map_waypoints_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> Maptool::getXY(double s, double d)
{
	int prev_wp = -1;

	while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%map_waypoints_x.size();

	double heading = atan2((map_waypoints_y[wp2]-map_waypoints_y[prev_wp]),(map_waypoints_x[wp2]-map_waypoints_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-map_waypoints_s[prev_wp]);

	double seg_x = map_waypoints_x[prev_wp]+seg_s*cos(heading);
	double seg_y = map_waypoints_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}


int Maptool::ClosestWaypoint(double x, double y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < map_waypoints_x.size(); i++)
	{
		double map_x = map_waypoints_x[i];
		double map_y = map_waypoints_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}


int Maptool::NextWaypoint(double x, double y, double theta)
{
	int closestWaypoint = ClosestWaypoint(x,y);

	double map_x = map_waypoints_x[closestWaypoint];
	double map_y = map_waypoints_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
	angle = std::min(2*M_PI - angle, angle);

	if(angle > M_PI/4)
	{
		closestWaypoint++;
		if (closestWaypoint == map_waypoints_x.size())
			closestWaypoint = 0;
	}

	return closestWaypoint;
}





std::vector<double> Maptool::parabolicGetXY(double s, double d)
{
	double max_s = 6945.554; //max s value for waypoints
	while (s > max_s)
		s -= max_s;

	int prev_wp = -1;

	// find the previous waypoint
	while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
		prev_wp++;

	int cubic_num; //helper index of the previous waypoint

	std::vector<double> X; // X coordinates of nearest waypoints used for interpolation
	std::vector<double> Y; // Y coordinates of nearest waypoints used for interpolation

	// fill X and Y with 1 previous waypoint and 2 successive waypoints,
	// if previous waypoint is 0 then start from the last waypoint in the map

	if (prev_wp >=1) 
	{
		cubic_num = 1;
		for (int i = -1; i < 3; i++)
		{
			X.push_back( map_waypoints_x[(prev_wp + i)%map_waypoints_x.size()] );
			Y.push_back( map_waypoints_y[(prev_wp + i)%map_waypoints_x.size()] );
		}
	} 
	else {
		cubic_num = 1;
		for (int i = map_waypoints_x.size() -1 ; i < map_waypoints_x.size() + 3; i++)
		{
			X.push_back( map_waypoints_x[i%map_waypoints_x.size()] );
			Y.push_back( map_waypoints_y[i%map_waypoints_x.size()] );
		}
	}

	double ds_p = s - map_waypoints_s[prev_wp]; //distance in s from previous waypoint

	std::vector<double> XYp = parabolicInterpol(X, Y, cubic_num+1, ds_p, d); // calculate x,y using the previous waypoint as the central waypoint for interpolation

	/*double ds_s; // distance in s from the next waypoint
	if (prev_wp == map_waypoints_s.size() - 1 )
		ds_s = s - max_s;  
	else
		ds_s = s - map_waypoints_s[(prev_wp+1)];  


	std::vector<double> XYs = parabolicInterpol(X,Y, cubic_num+1, ds_s, d); // calculate x,y using the next waypoint as the central waypoint for interpolation

	// calculate the weighted mean of the two interpolations using the inverse sqaure of the distance from previous and next waypoint
	int n_exp = -2;
	double p1 = pow(ds_p,n_exp);
	double p2 = pow(ds_s,n_exp);
	double norm =p1+p2;
	double x = (XYp[0]*p1 + XYs[0]*p2)/(norm);
	double y = (XYp[1]*p1 + XYs[1]*p2)/(norm);
	return {x,y};*/

	return {XYp[0],XYp[1]};
	

}



std::vector<double> Maptool::parabolicInterpol(std::vector<double> X, std::vector<double> Y, int center, double ds, double d) 
{
	std::vector<std::vector<double>> Points_global(2);

	Points_global[0].push_back( X[center-1] );
	Points_global[1].push_back( Y[center-1] );

	Points_global[0].push_back( X[center] );
	Points_global[1].push_back( Y[center] );

	Points_global[0].push_back( X[center+1] );
	Points_global[1].push_back( Y[center+1] );

	//Points_global[0].push_back( X[center+2] );
	//Points_global[1].push_back( Y[center+2] );

	double ybla = Y[center+1] - Y[center-1];
	double xbla = X[center+1] - X[center-1];

	double angle = atan2(ybla, xbla);

	global2local(Points_global, angle, X[center-1], Y[center-1]);

	// Calculate splines:
	tk::spline spl;
	spl.set_points(Points_global[0], Points_global[1]);

	double dist1 = distance(X[center], Y[center], X[center-1], Y[center-1]);
	double dist2 = distance(X[center+1], Y[center+1], X[center], Y[center]);

	double ratio = ds / (dist1 + dist2);

	double xp = ratio * distance(X[center+1], Y[center+1], X[center-1], Y[center-1]);
	double yp = spl(xp);
	double heading = atan(spl.deriv(1, xp));

	heading = heading - M_PI;
	xp += -d*sin(heading);
	yp += d*cos(heading);

	//std::cout << "xp = " << xp << "\t" << "yp = "<< yp << std::endl;
	//std::cout << heading << std::endl;

	// transform back to global coordinates
	std::vector<std::vector<double>> asd(2);
	asd[0].push_back(xp);
	asd[1].push_back(yp);
	local2global(asd, angle, X[center-1], Y[center-1]);

	return {asd[0][0], asd[1][0]};
}







