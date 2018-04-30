#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include "maptool.h"
#include <iostream>
#include "spline.h"

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


std::vector<double> Maptool::parabolicInterpol(std::vector<double> X, std::vector<double> Y, int center, double ds, double d) 
{
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


	double X1 = X[center]-X[center]; // transform to reference system of center point
	double X2 = X[center+abs(ds)/ds]-X[center]; // second integration limit is the previous or successive point of center, according to ds sign    

	std::vector<std::vector<double>> anchor_vals(2);

	anchor_vals[0].push_back( P0[0] );
	anchor_vals[1].push_back( P0[1] );

	anchor_vals[0].push_back( P1[0] );
	anchor_vals[1].push_back( P1[1] );

	anchor_vals[0].push_back( P2[0] );
	anchor_vals[1].push_back( P2[1] );

	// Calculate splines:
	tk::spline spl;
	spl.set_points(anchor_vals[0], anchor_vals[1]);

	double dist1 = distance(P1[0], P1[1], P0[0], P0[1]);
	double dist2 = distance(P2[0], P2[1], P1[0], P0[0]);

	double ratio0 = ds / (dist1 + dist2);

	double xp = ratio0*(P2[0]-P1[0]);
	double yp = spl(xp);




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









/*vector<vector<double>> StateMachine::generate_trajectory(state proposed_state, pose egoPose, vector<vector<double>> target_vehicles)
{
	// Define anchor points here:
	vector<vector<double>> anchor_vals(2);
	vector<vector<double>> trajectory(2);

	// Define "anchor waypoints" 30, 60, and 90 meters in front of the car:
	anchor_vals[0].push_back( egoPose.pos_x );
	anchor_vals[1].push_back( egoPose.pos_y );

	double d_1, d_2, d_3;

	d_1 = 4*intended_lane+2;
	d_2 = 4*intended_lane+2;
	d_3 = 4*intended_lane+2;

	vector<double> tmp(2);
	tmp = maptool.getXY(egoPose.s+30, d_1);
	anchor_vals[0].push_back( tmp[0] );
	anchor_vals[1].push_back( tmp[1] );
	tmp = maptool.getXY(egoPose.s+60, d_2);
	anchor_vals[0].push_back( tmp[0] );
	anchor_vals[1].push_back( tmp[1] );
	tmp = maptool.getXY(egoPose.s+90, d_3);
	anchor_vals[0].push_back( tmp[0] );
	anchor_vals[1].push_back( tmp[1] );


	// Target speed (in m/s):
	const double target_inc = 0.427;

	bool vehicle_ahead = false;

    for (int i=0; i<target_vehicles.size(); i++)
    {
	    double target_d = target_vehicles[i][6];
	    // Check if target vehicle is on the same lane
	    if ( target_d > (d_3-2) && target_d < (d_3+2) )
	    {
	        double target_s = target_vehicles[i][5];
			double target_vx = target_vehicles[i][3];
	        double target_vy = target_vehicles[i][4];
	        double target_speed = sqrt(target_vx*target_vx+target_vy*target_vy);

	        // Check is target vehicle is on collision course
	        if ( (egoPose.s-target_s)<50 && egoPose.s-target_s>0 && target_speed <= 50*dist_inc)
	        	vehicle_ahead = true;
	    }
	}


	if (vehicle_ahead) {
		dist_inc -= 0.02;
		cout << "Vehicle ahead!" << endl;
	}
	else if (dist_inc <= target_inc)
		dist_inc += 0.04;


	global2vehicle(anchor_vals, egoPose);
	
	//for (int i=0; i<anchor_vals[0].size(); i++)
	//    cout << "anchor_vals: " << anchor_vals[0][i] << "\t" << anchor_vals[1][i] << endl;

	// Calculate splines:
	tk::spline spl;
	spl.set_points(anchor_vals[0], anchor_vals[1]);

	// Generate trajectory:
	double dist = distance(0, 0, 30, spl(30));
	int N = int (dist / dist_inc);

	// In order to eliminate jiggering in longitudinal direction.
	for(int i=1; i-1<150-remaining_points; i++)
	{		      
		trajectory[0].push_back(i*dist_inc);
		trajectory[1].push_back(spl(i*dist_inc));
	}

	vehicle2global(trajectory, egoPose);

	return trajectory;
}*/

