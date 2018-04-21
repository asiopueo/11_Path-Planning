#include <cmath>
#include "statemachine.h"
#include "maptool.h"
#include "spline.h"
#include <iostream>

StateMachine::StateMachine()
{
	current_state = LK;
	possible_successor_states[LK]=LK;
	//possible_successor_states["STAY"].push_back(PLCL);
	//possible_successor_states["STAY"].push_back(PLCR);
	//possible_successor_states["PLCL"].push_back();
	//possible_successor_states["SLCR"].push_back();
	//possible_successor_states["LCL"].push_back();
	//possible_successor_states["LCR"].push_back();

	weight_1 = 1.0;
	weight_2 = 1.0;


}

StateMachine::~StateMachine()
{

}

void successor_states()
{
}




vector<vector<double>> StateMachine::generate_trajectory(state proposed_state, pose ego_veh, auto target_vehicles)
{
	switch(proposed_state)
	{
		case LK:
			// Define anchor points here:
		    vector<vector<double>> anchor_vals(2);
		    vector<vector<double>> trajectory(2);
		    vector<double> egoFrenet = maptool.getFrenet(ego_veh.pos_x, ego_veh.pos_y, ego_veh.angle);
		    
		    // Define "anchor waypoints" 30, 60, and 90 meters in front of the car:
		    vector<double> tmp = maptool.getXY(ego_veh.s+30, 4*ego_veh.lane+2);
		    anchor_vals[0].push_back( tmp[0] );
		    anchor_vals[1].push_back( tmp[1] );
		    tmp = maptool.getXY(ego_veh.s+60, 4*ego_veh.lane+2);
		    anchor_vals[0].push_back( tmp[0] );
		    anchor_vals[1].push_back( tmp[1] );
		    tmp = maptool.getXY(ego_veh.s+90, 4*ego_veh.lane+2);
		    anchor_vals[0].push_back( tmp[0] );
		    anchor_vals[1].push_back( tmp[1] );

		    global2vehicle(anchor_vals, ego_veh);

		    // Calculate splines:
		    tk::spline spl;
		    spl.set_points(anchor_vals[0], anchor_vals[1]);
		    
		    // Generate trajectory:
		    double dist_inc = 0.447;

			double dist = distance(ego_veh.pos_x, ego_veh.pos_y, anchor_vals[0][2], anchor_vals[1][2]);

		    for(int i=0; i<50; i++)
		    {		      
		      trajectory[0].push_back(i*dist/50);
		      trajectory[1].push_back(spl(i*dist/50));
		    }

		    vehicle2global(trajectory, ego_veh);
		    return trajectory;

		case PLCL:
			;

		case LCL:
			;

		case PLCR:
			;

		case LCR:
			;

		case EA:
			;

	}

}



double StateMachine::cost_function_1(vector<vector<double>> trajectory)
{
	double cost;

	for(int i=0; i<trajectory[0].size(); i++)
	{
		//cost += function(trajectory[0][i], trajectory[1][i]);
	}

	return cost;
}



double StateMachine::cost_function_2(vector<vector<double>> trajectory)
{
	double cost;

	for(int i=0; i<trajectory[0].size(); i++)
	{
		//cost += function(trajectory[0][i], trajectory[1][i]);
	}

	return cost;
}



void StateMachine::evaluate_behavior(pose ego_veh, vector<vector<double>> vehicle_list)
{
	state best_next_state;
    unsigned int min_cost = UINT_MAX;
    vector<vector<double>> projected_trajectory(2);
    map<state, double> costs;

    //cout << "Current state: " << current_state << endl;
    for (auto iter : vehicle_list)
    	cout << iter[0] << endl;
    /*for(state_iter : possible_successor_states[current_state])
    {
        projected_trajectory = generate_trajectory(state_iter, current_pose, predictions);
        double cost_for_state = 0;

        cost_for_state += weight_1 * cost_function_1(projected_trajectory, predictions);
        //cost_for_state += weight_2 * cost_function_2(projected_trajectory, predictions);

        costs.insert(std::make_pair(state_iter, cost_for_state));
    }

    for(auto iter : possible_successor_states[current_state]) {
        if(costs[iter] < min_cost) {
            min_cost = costs[iter];
            best_next_state = state;
		}
	}*/
}



void StateMachine::execute_state_transition()
{
}




    /*
        double d = sensor_fusion[i][6];
        // Check if target vehicle is on the same lane
        if ( d>(4*ego_pose.lane) && d<(4*ego_pose.lane+4) )
        {
            double target_s = sensor_fusion[i][5];
            // Check is target vehicle is on collision course
            if ( (target_s-ego_pose.s)<30 && target_s>ego_pose.d-30)
            {
                //double target_vx = sensor_fusion[i][3];
                //double target_vy = sensor_fusion[i][4];
                //double target_speed = sqrt(target_vx*target_vx+target_vy*target_vy);
                vehicle_list.push_back(sensor_fusion[i]);
            }
        }
    */

