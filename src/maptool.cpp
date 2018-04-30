#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include "maptool.h"
#include <iostream>


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
	// This functions transform from s,d coordinates to global x,y coordinates using a waypoint maps of the highway
	// Instead of a linear interpolation, it uses two parabolic interpolation and then calculates a weighted mean. 
	// The first interpolation is made using the previous waypoint and the immidiately successive and previous waypoint,
	// the second interpolation is made using the previous waypoint and the immidiately 2 successive waypoints.
	// Then a weighted mean of the two points is calculated using, as weights, the inverse of the squared distance from the 
	// previous waypoint and the next waypoint

	//INPUT:
	// s coordinate
	// d coordinate
	// s values of waypoints
	// x values of waypoints
	// y values of waypoints

	//OUTPUT:
	// a vector of (x,y) coordinate for point (s,d)

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

	std::vector<double> XYp = parabolicInterpol(X, Y, cubic_num, ds_p, d); // calculate x,y using the previous waypoint as the central waypoint for interpolation

	double ds_s; // distance in s from the next waypoint
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
	return {x,y};
}  


double Maptool::calcPoly(std::vector<double> coeffs, double t) 
{
	// This functions calculates the value of a polynomial at time t and returns its value

	//INPUT:
	// a vector of all coefficients for the polynomial sorted from lowest degree to highest
	// the time t at which evaluate the polynomial

	//OUTPUT:
	// the value of the polynomial at time t

	double pol = 0.;
	for (int i = 0; i < coeffs.size(); i++)
		pol += coeffs[i] * pow(t, i);

	return pol;
}



std::vector<double> Maptool::parabolicInterpol(std::vector<double> X, std::vector<double> Y, int center, double ds, double d) 
{
	// This functions interpolates a 2nd grade polynomial between 3 waypoints X,Y and then uses ds and d to estimate the x,y position
	// of a point between the center waypoint and the next

	//INPUT:
	// vector of X coordinates of 3 waypoints
	// vector of Y coordinates of 3 waypoints
	// index of the central waypoint
	// arc lenght along the parabola ds
	// coordinate d 

	//OUTPUT:
	// a vector of (x,y) coordinate for point (ds,d)

	// transform to reference system of center point
	double x0 = X[center-1] - X[center]; 
	double x1 = X[center] - X[center];
	double x2 = X[center+1] - X[center];

	double y0 = Y[center-1] - Y[center];
	double y1 = Y[center] - Y[center];
	double y2 = Y[center+1] - Y[center];

	double den_X = (x0-x1)*(x0-x2)*(x1-x2);
	double den_y = (y0-y1)*(y0-y2)*(y1-y2);
	double disc_x = (x0-x1)*(x1-x2);
	double disc_y = (y0-y1)*(y1-y2);
	bool rotate = false;

	if (disc_x <= 0 )  
	{
		//rotate reference system, so that (x,y) -> (-y,x)
		double tx0 = -y0;
		double tx1 = -y1;
		double tx2 = -y2;

		y0 = x0;
		y1 = x1;
		y2 = x2;

		x0 = tx0;
		x1 = tx1;
		x2 = tx2;

		std::vector<double> TX;

		for (int i =0; i<X.size(); i++)
		{
			TX.push_back(-Y[i]);
			Y[i]=X[i];
			X[i]=TX[i];
		}

		rotate = true;
	}

	// Calculate 3 parameters of the parabola passing by the 3 waypoints y=ax^2+bx+c
	double den = (x0-x1)*(x0-x2)*(x1-x2);
	double a = ( x2*(y1-y0) + x1*(y0-y2) + x0*(y2-y1) )/den;
	double b = ( x2*x2*(y0-y1) + x1*x1*(y2-y0) +x0*x0*(y1-y2) )/den;
	double c = ( x1*x2*(x1-x2)*y0 + x2*x0*(x2-x0)*y1 +x0*x1*(x0-x1)*y2 )/den;


	double sum = 0.;
	int idx = 0;

	double X1 = X[center]-X[center]; // transform to reference system of center point
	double X2 = X[center+abs(ds)/ds]-X[center]; // second integration limit is the previous or successive point of center, according to ds sign    

	double h = (X2-X1)/5000.;

	// the arc lenght of a parabola is the definite integral of sqrt(1+f'(x)^2) in dx
	double u1 = 2.*a*X1 + b; // helper variable 
	double g1 = u1*sqrt(1+u1*u1) + log(abs(sqrt(1+u1*u1) + u1)); // primitive of sqrt(1+f'(x)^2) calculated in X1

	double xe2=X1;

	// EVALUATE xe2 at which the arc lenght equals |ds| with 1e-11 tolerance or with 10000000 max iterations, whatever happens first
	while (( abs(abs(ds) - sum) > 1e-100) && (idx < 100))
	{
		xe2 += h;
		double u2 = 2.*a*xe2 + b;
		double g2 = (u2*sqrt(1+u2*u2) + log(abs(sqrt(1+u2*u2) + u2))); // primitive of sqrt(1+f'(x)^2) calculated in xe2

		sum = abs((g2 - g1)/(4.*a)); // arc lenght from X1 to xe2
		if (sum > abs(ds) ) // if arc lenght is greater than |ds| go back one step and divide h by 2
		{
			xe2 -= h;  
			h = h/2.;  
		}
		idx++;
	}

	double xp = xe2;
	double yp = calcPoly({c,b,a},xp);
	double heading = atan2(2.*a*xp + b, 1.); //calculate heading of parabola at point (xp, yp=2axp+b)

	// transform back to global reference system
	xp += X[center];
	yp += Y[center];

	if (rotate)
	{
		//rotate back
		double txp= xp;
		xp = yp;
		yp = -txp;


		if (x1-x0 > 0.)
		{
			heading = heading + M_PI;
		}

		// add d offset using heading

		xp += d * cos(heading);
		yp += d * sin(heading);
	} 
	else 
	{
		if (x1-x0 < 0.)
		{
			heading = heading + M_PI;
		}
		heading = heading-M_PI/2.;
		xp += d * cos(heading);
		yp += d * sin(heading);  
	}

	return{xp,yp};
}