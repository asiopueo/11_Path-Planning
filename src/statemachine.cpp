#include <cmath>
#include "statemachine.h"
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// Only necessary when fooling around with splines:
// #include "spline.h"





StateMachine::StateMachine()
{
	current_state = LK;

	// Successor states of Lane Keep:
	successor_states[LK].push_back(LK);
	successor_states[LK].push_back(LCL);
	successor_states[LK].push_back(LCR);

	// Successor states of Lane Change Left:
	successor_states[LCL].push_back(LCL);
	successor_states[LCL].push_back(LK);

	// Successor states of Lane Change Right:
	successor_states[LCR].push_back(LCR);
	successor_states[LCR].push_back(LK);


	dist_inc = 0.0;

	weights.push_back(1.0);
	weights.push_back(1.0);

	current_lane = 1;
	intended_lane = 1;

}

StateMachine::~StateMachine() {
}

void successor_states() {
}



/*
 *	TODO: getXY() needs to be reimplemented as the polygonal vertices (and corresponding jerks) arise from there!
 */



std::vector<std::vector<double>> StateMachine::generate_trajectory(state proposed_state, pose ego_veh, std::vector<std::vector<double>> target_vehicles)
{
	switch(proposed_state) {
		case LK:
			intended_lane = current_lane;
			break;
		case LCL:
			intended_lane = current_lane-1;
			break;
		case LCR:
			intended_lane = current_lane+1;
			break;
	}


	std::vector<std::vector<double>> trajectory(2);

	bool vehicle_ahead = false;
    for (int i=0; i<target_vehicles.size(); i++)
    {
	    double target_d = target_vehicles[i][6];
	    // Check if target vehicle is on the same lane
	    if ( target_d > (4*current_lane-2) && target_d < (4*current_lane+2) )
	    {
	        double target_s = target_vehicles[i][5];
			double target_vx = target_vehicles[i][3];
	        double target_vy = target_vehicles[i][4];
	        double target_speed = sqrt(target_vx*target_vx+target_vy*target_vy);

	        // Check is target vehicle is on collision course
	        if ( (ego_veh.s-target_s)<50 && ego_veh.s-target_s>0 && target_speed <= 50*dist_inc)
	        	vehicle_ahead = true;
	    }
	}


	double T = 4.2; // Total time for trajectory [sec]
	unsigned int number_of_steps;


	if (vehicle_ahead) {
		T += 0.02;
		std::cout << "Vehicle ahead!" << std::endl;
	}
	//else if (dist_inc <= 10)
	//	T -= 0.02;





	// Calculation for s(t):
	Eigen::MatrixXd matrix_s = Eigen::MatrixXd(3,3);
	Eigen::VectorXd vector_s = Eigen::VectorXd(3);
	Eigen::VectorXd s_i = Eigen::VectorXd(3);
	Eigen::VectorXd s_f = Eigen::VectorXd(3);

	matrix_s << pow(T,3), pow(T,4), pow(T,5),
				3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
				6*pow(T,1), 12*pow(T,2), 20*pow(T,3);

	s_i << ego_veh.s, 		5., 0.;
	s_f << ego_veh.s + 90., 5., 0.;

	vector_s << s_f(0) - (s_i(0)+s_i(1)*T+0.5*s_i(2)*pow(T,2)), 
				s_f(1) - (s_i(1)+s_i(2)*T), 
				s_f(2) -  s_i(2);

	// Calculation for d(t):
	Eigen::MatrixXd matrix_d = Eigen::MatrixXd(3,3);
	Eigen::VectorXd vector_d = Eigen::VectorXd(3);
	Eigen::VectorXd d_i = Eigen::VectorXd(3);
	Eigen::VectorXd d_f = Eigen::VectorXd(3);

	matrix_d << pow(T,3), pow(T,4), pow(T,5),
				3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
				6*pow(T,1), 12*pow(T,2), 20*pow(T,3);

	d_i << ego_veh.d, 0., 0.;
	d_f << 4*intended_lane+2, 0., 0.;

	vector_d << d_f(0) - (d_i(0)+d_i(1)*T+0.5*d_i(2)*pow(T,2)), 
				d_f(1) - (d_i(1)+d_i(2)*T), 
				d_f(2) -  d_i(2);


	// Result of these calculations are the coefficients \alpha_3, \alpha_4, and \alpha_5
	Eigen::VectorXd result_s = calculate_coefficients(matrix_s, vector_s);
	Eigen::VectorXd result_d = calculate_coefficients(matrix_d, vector_d);

	Eigen::VectorXd coeffs_s = Eigen::VectorXd(6);
	Eigen::VectorXd coeffs_d = Eigen::VectorXd(6);

	coeffs_s << s_i(0), s_i(1), 0.5*s_i(2), result_s(0), result_s(1), result_s(2);
	coeffs_d << d_i(0), d_i(1), 0.5*d_i(2), result_d(0), result_d(1), result_d(2);


	// Prefactor of 50 accounts for 50Hz/20ms update frequency
	number_of_steps = T / 0.02;

	//for (int i=0; i<6; i++)
	//	std::cout << coeffs_s(i) << std::endl;

	// In order to eliminate jiggering in longitudinal direction.
	for(int i=1; i < number_of_steps; i++)
	{		      
		trajectory[0].push_back(quintic_poly(coeffs_s, 0.02*i ));
		trajectory[1].push_back(quintic_poly(coeffs_d, 0.02*i ));
	}


	for (int i=0; i<trajectory[0].size(); i++) 
	{
		std::vector<double> tmpXY;
		tmpXY = maptool.getXY(trajectory[0][i], trajectory[1][i]);
		trajectory[0][i] = tmpXY[0];
		trajectory[1][i] = tmpXY[1];
	}

	return trajectory;
}



double StateMachine::cost_function_1(std::vector<std::vector<double>> trajectory, std::vector<std::vector<std::vector<double>>> list)
{
	double cost;
	unsigned int end_of_path = trajectory[0].size()-1;

	/*double target_s = target_vehicles[i][5];
	double target_vx = target_vehicles[i][3];
	double target_vy = target_vehicles[i][4];
	double target_speed = sqrt(target_vx*target_vx+target_vy*target_vy);

	// Check is target vehicle is on collision course
	if ( (ego_veh.s-target_s)<50 && ego_veh.s-target_s>0 && target_speed <= 50*dist_inc)
	vehicle_ahead = true;
	*/


		//delta_s = ego_veh.s - vehicle.s;
	double delta_d = abs(intended_lane-current_lane);
	// The faster the trajectory is, the better!
	cost = exp( -trajectory[0][end_of_path] / delta_d );
	//std::cout << "Cost: " << cost << std::endl;
	return cost;
}





std::vector<std::vector<double>> StateMachine::evaluate_behavior(pose ego_veh, std::vector<std::vector<double>> vehicle_list, int rest)
{
	state best_next_state;
	remaining_points = rest;
    unsigned int min_cost = UINT_MAX;

	std::vector<std::vector<double>> trajectory(2);
    //std::map<state, double> costs;

    //std::cout << "Current state: " << current_state << std::endl;
    //for (auto iter : vehicle_list)
    //	cout << iter[0] << endl;

	// Frenet coordinates of ego-vehicle:
    std::vector<double> egoFrenet = maptool.getFrenet(ego_veh.pos_x, ego_veh.pos_y, ego_veh.angle);
    ego_veh.s = egoFrenet[0];
    ego_veh.d = egoFrenet[1];

    if (abs(ego_veh.s-4*intended_lane+2)<0.1) {
    	current_lane = intended_lane;
    	current_state = LK;
    }

    for(auto state_iter : successor_states[current_state])
    {
	    double cost_for_state = 0;
		bool vehicle_ahead = false;

	    // Check if there is a vehicle on the left lane:
	    // Make the cost very high if the lane is occupied!
		std::vector<std::vector<std::vector<double>>> vehicles(3);

	    for (auto target : vehicle_list)
	    {
		    double target_d = target[6];

		    // Check on which lane the target vehicle is:
		    if ( 0< target_d <4 )
		        vehicles[0].push_back(target);
		    else if (4 <= target_d <= 8)
		    	vehicles[1].push_back(target);
		    else if (8 < target_d < 12)
		    	vehicles[2].push_back(target);
		    // else: Vehicles not on the road will be ignored
		}

		associated_trajectories[state_iter] = generate_trajectory(state_iter, ego_veh, vehicle_list);
		cost_for_state = cost_function_1(associated_trajectories[state_iter], vehicles);

  		if (cost_for_state < min_cost){
  			min_cost = cost_for_state;
  			best_next_state = state_iter;
  		}
	}
	std::cout << "LCL:" << "\tLK:" << "\tLCR:" << std::endl;

	return trajectory = associated_trajectories[best_next_state];
}




/*
 *	Old code from playing around with splines:
 */

/*vector<vector<double>> StateMachine::generate_trajectory(state proposed_state, pose ego_veh, vector<vector<double>> target_vehicles)
{
	// Define anchor points here:
	vector<vector<double>> anchor_vals(2);
	vector<vector<double>> trajectory(2);

	// Define "anchor waypoints" 30, 60, and 90 meters in front of the car:
	anchor_vals[0].push_back( ego_veh.pos_x );
	anchor_vals[1].push_back( ego_veh.pos_y );

	double d_1, d_2, d_3;

	d_1 = 4*intended_lane+2;
	d_2 = 4*intended_lane+2;
	d_3 = 4*intended_lane+2;

	vector<double> tmp(2);
	tmp = maptool.getXY(ego_veh.s+30, d_1);
	anchor_vals[0].push_back( tmp[0] );
	anchor_vals[1].push_back( tmp[1] );
	tmp = maptool.getXY(ego_veh.s+60, d_2);
	anchor_vals[0].push_back( tmp[0] );
	anchor_vals[1].push_back( tmp[1] );
	tmp = maptool.getXY(ego_veh.s+90, d_3);
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
	        if ( (ego_veh.s-target_s)<50 && ego_veh.s-target_s>0 && target_speed <= 50*dist_inc)
	        	vehicle_ahead = true;
	    }
	}


	if (vehicle_ahead) {
		dist_inc -= 0.02;
		cout << "Vehicle ahead!" << endl;
	}
	else if (dist_inc <= target_inc)
		dist_inc += 0.04;


	global2vehicle(anchor_vals, ego_veh);
	
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

	vehicle2global(trajectory, ego_veh);

	return trajectory;
}*/

