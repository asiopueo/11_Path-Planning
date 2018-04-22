#include <cmath>
#include "statemachine.h"
#include "maptool.h"
#include "spline.h"
#include <iostream>

StateMachine::StateMachine()
{
	current_state = LK;
	successor_states[LK]=LK;
	//successor_states["STAY"].push_back(PLCL);
	//successor_states["STAY"].push_back(PLCR);
	//successor_states["PLCL"].push_back();
	//successor_states["SLCR"].push_back();
	//successor_states["LCL"].push_back();
	//successor_states["LCR"].push_back();

	dist_inc = 0.0;

	weight_1 = 1.0;
	weight_2 = 1.0;

	current_lane = 1;
	intended_lane = 1;


}

StateMachine::~StateMachine()
{

}

void successor_states()
{
}




vector<vector<double>> StateMachine::generate_trajectory(state proposed_state, pose ego_veh, vector<vector<double>> target_vehicles)
{
	// Define anchor points here:
	vector<vector<double>> anchor_vals(2);
	vector<vector<double>> trajectory(2);
	

	// Define "anchor waypoints" 30, 60, and 90 meters in front of the car:
	anchor_vals[0].push_back( ego_veh.pos_x );
	anchor_vals[1].push_back( ego_veh.pos_y );

	double d_1, d_2, d_3;

	switch(proposed_state)
	{
		case LK:
			d_1 = 4*intended_lane+2;
			d_2 = 4*intended_lane+2;
			d_3 = 4*intended_lane+2;
			break;

		case PLCL:
			d_1 = 4*intended_lane+2;
			d_2 = 4*intended_lane;
			d_3 = 4*intended_lane-2;
			break;

		case PLCR:
			d_1 = 4*intended_lane+2;
			d_2 = 4*intended_lane+4;
			d_3 = 4*intended_lane+6;
			break;

		case LCL:
			d_1 = 4*intended_lane+2;
			d_2 = 4*intended_lane;
			d_3 = 4*intended_lane-2;
			break;

		case LCR:
			d_1 = 4*intended_lane+2;
			d_2 = 4*intended_lane+4;
			d_3 = 4*intended_lane+6;
			break;

		case EA:
			d_1 = 4*intended_lane+2;
			d_2 = 4*intended_lane+2;
			d_3 = 4*intended_lane+2;
			break;
	}


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
}



double StateMachine::cost_function_1(vector<vector<double>> trajectory)
{
	double cost;

	unsigned int end_of_path = trajectory[0].size();

	cost = trajectory[0][end_of_path];

	return cost;
}



double StateMachine::cost_function_2(vector<vector<double>> trajectory)
{
	double cost;

	//cost = function(trajectory[0][i], trajectory[1][i]);

	return cost;
}



std::vector<std::vector<double>> StateMachine::evaluate_behavior(pose ego_veh, vector<vector<double>> vehicle_list, int rest)
{
	state best_next_state;
	remaining_points = rest;
    unsigned int min_cost = UINT_MAX;

	vector<vector<double>> trajectory(2);
    vector<vector<double>> trajectory_LK(2);
    vector<vector<double>> trajectory_LCL(2);
    vector<vector<double>> trajectory_LCR(2);
    map<state, double> costs;

    cout << "Current state: " << current_state << endl;
    //for (auto iter : vehicle_list)
    //	cout << iter[0] << endl;

    vector<double> egoFrenet = maptool.getFrenet(ego_veh.pos_x, ego_veh.pos_y, ego_veh.angle);
    ego_veh.s = egoFrenet[0];
    ego_veh.d = egoFrenet[1];

    if (abs(ego_veh.s-4*intended_lane+2)< 0.1) {
    	current_lane = intended_lane;
    	current_state = LK;
    }



    //for(state_iter : successor_states[current_state])
    {
    	trajectory_LK = generate_trajectory(LK, ego_veh, vehicle_list);
    	if (current_lane != 0)
    		trajectory_LCL = generate_trajectory(LCL, ego_veh, vehicle_list);
    	else if (current_lane != 2)
    		trajectory_LCR = generate_trajectory(LCR, ego_veh, vehicle_list);
	    
	    double cost_for_state = 0;
        //cost_for_state += weight_1 * cost_function_1(projected_trajectory, vehicle_list);
	    //cost_for_state += weight_2 * cost_function_2(projected_trajectory, predictions);

        //costs.insert(std::make_pair(state_iter, cost_for_state));
	    costs[LK] = 0.;
	    costs[LCL] = 0.;
	    costs[LCR] = 0.;


	    // Check if there is a vehicle on the left lane:
	    // Make the cost very high if the lane is occupied!
	    for (int i=0; i<target_vehicles.size(); i++)
	    {
		    double target_d = target_vehicles[i][6];
		    // Check if target vehicle is on the same lane
		    if ( target_d > (4*intended_lane) && target_d < (4*intended_lane+4) )
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

		



	    /*for(auto iter : possible_successor_states[current_state]) 
	    {
	        if(costs[iter] < min_cost) {
	            min_cost = costs[iter];
	            best_next_state = state;
		
	        if (LCL)
	    		intended_lane = lane-1;
	    	else if (LCR)
	    		intended_lane = lane+1;
		}*/

	   

	    trajectory = trajectory_LK;
	}
	return trajectory;
}



void StateMachine::execute_state_transition()
{
}






