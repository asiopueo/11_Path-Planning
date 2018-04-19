#include "statemachine.h"


StateMachine::StateMachine()
{
	current_state = Stay;

	possible_successor_states["STAY"].push_back(PLCL);
	possible_successor_states["STAY"].push_back(PLCR);
	//possible_successor_states["PLCL"].push_back();
	//possible_successor_states["SLCR"].push_back();
	//possible_successor_states["LCL"].push_back();
	//possible_successor_states["LCR"].push_back();

	weight_1;
	weight_2;
}

StateMachine::~StateMachine()
{

}

void successor_states()
{
	return;
}




vector<vector<double>> generate_trajectory(state, pose, vehicle_list)
{
    // Define anchor points here:
    vector<double> anchor_x_vals(3);
    vector<double> anchor_y_vals(3);
    vector<double> egoFrenet = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
    
    // Define "anchor waypoints" 30, 60, and 90 meters in front of the car:
    vector<double> tmp = getXY(egoFrenet[0]+30, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    anchor_x_vals.push_back( tmp[0] );
    anchor_y_vals.push_back( tmp[1] );
    tmp = getXY(egoFrenet[0]+60, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    anchor_x_vals.push_back( tmp[0] );
    anchor_y_vals.push_back( tmp[1] );
    tmp = getXY(egoFrenet[0]+90, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    anchor_x_vals.push_back( tmp[0] );
    anchor_y_vals.push_back( tmp[1] );

    global2vehicle_coords();

    // Calculate splines:
    tk::spline spl;
    spl.set_points(X,Y);
    
    // Generate trajectory:
    for(int i=0; i<50-path_size; i++)
    {
      dist = distance(pos_x, pos_y, anchor_x, anchor_y);
      N = dist/speed;
      y = spl(x);
    }

    vehicle2global_coords();

}



double cost_function_1(vector<vector<double>> trajectory)
{
	for(int i=0; i<trajectory(0).size(); i++)
	{
		cost += function(trajectory[0][i], trajectory[1][i]);
	}


	return cost;
}



double cost_function_2(vector<vector<double>> trajectory)
{
	for(int i=0; i<trajectory(0).size(); i++)
	{
		cost += function(trajectory[0][i], trajectory[1][i]);
	}

	return cost;
}



void StateMachine::evaluate_behavior(current_pose, predictions)
{
	state best_next_state;
    unsigned int min_cost = UINT_MAX;
    vector<vector<double>> projected_trajectory(2);
    map<state, double> costs;

    for(state_iter : possible_successor_states[current_state])
    {
        projected_trajectory = generate_trajectory(state_iter, current_pose, predictions);
        double cost_for_state = 0;

        cost_for_state += weight_1 * cost_function_1(projected_trajectory, predictions);
        cost_for_state += weight_2 * cost_function_2(projected_trajectory, predictions);

        costs.insert(std::make_pair(state_iter, cost_for_state));
    }

    for(auto iter : possible_successor_states[current_state]) {
        if(costs[iter] < min_cost) {
            min_cost = costs[iter];
            best_next_state = state;
		}
	}
}



void StateMachine::execute_state_transition()
{
	return best_trajectory;
}





	// somewhere inside th cost functions...
    // sensor_fusion data format: []
    for (int i=0; i< sensor_fusion.size(); ++i)
    {
      float d = sensor_fusion[i][6];

      // Check if target vehicle is on the same lane
      if ( d>(4*lane) && d<(4*lane+4) )
      {
        double target_vx = sensor_fusion[i][3];
        double target_vy = sensor_fusion[i][4];
        double target_speed = sqrt(target_vx*target_vx+target_vy*target_vy);
        double target_s = sensor_fusion[i][5];

        // Check is target vehicle is on collision course
        if ( (target_s - egoFrenet[0])<30 && target_s>egoFrenet[0])
        {
          // Crash mitigation logic here!

          double ref_vel = 29.5;
        }


      }
    }

