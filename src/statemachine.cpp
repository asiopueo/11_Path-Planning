#include "statemachine.h"

#include <random>
#include <cmath>

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

	weights.push_back(1.0);
	weights.push_back(1.0);

	current_lane = 1;
	intended_lane = 1;

}

StateMachine::~StateMachine() {
}



// Costs due to near-collision with other vehicles
double StateMachine::cost_function_traffic(Trajectory &trajectory, const pose egoPose, const targetList_t list, Maptool map)
{
	double cost = 0;

	// Check if there is a vehicle on the left lane:
	// Make the cost very high if the lane is occupied!
	for (int i=0; i<list.size(); i++)
	{
		double target_id = list[i][0];
		/*double target_x = list[i][1];
		double target_y = list[i][2];
		double target_vx = list[i][3];
		double target_vy = list[i][4];*/
		double target_s = list[i][5];
		double target_d = list[i][6];
		
		//double target_speed = sqrt(target_vx*target_vx+target_vy*target_vy);
		//std::cout << target_id << "\t" << target_x << "\t" << target_y << std::endl;

		double dist_s = target_s-egoPose.s;

		if (4*intended_lane <= target_d && target_d <= 4*intended_lane+4)
		{
			if (-20 <= dist_s && dist_s <= 0)
				cost += 30;
			else if (0 <= dist_s && dist_s <= 30)
				cost += 30;
			else if (30 <= dist_s && dist_s <= 100)
				cost += 15;
			else
				cost += 0;
		}

		if (4*current_lane <= target_d && target_d <= 4*current_lane+4)
		{
			if (-20 <= dist_s && dist_s <= 0)
				cost += 60;
			else if (0 <= dist_s && dist_s <= 30)
				cost += 60;
			else if (30 <= dist_s && dist_s <= 100)
				cost += 30;
			else
				cost += 0;
		}

		//std::cout << target_id << ":\t" << dist_s << "\t" << cost << std::endl;
	}
	return cost;
}

// Costs due to trajectory dynamics
double StateMachine::cost_function_behavior(const Trajectory &trajectory, double ds, double dd)
{
	double cost = 0;

	// The faster the trajectory is, the better!
	cost += 5*exp(-ds);

	// Penalty for lane change
	cost += 5*abs(intended_lane - current_lane);
	//std::cout << "Cost: " << cost << std::endl;

	return cost;
}



std::vector<double> StateMachine::lane_speeds_fct(const pose egoPose, targetList_t list)
{
	std::vector<double> lane_list = {0.,0.,0.};
	std::vector<int> car_counter = {0,0,0};

	for (int i=0; i<list.size(); i++)
	{
		double target_id = list[i][0];
		double target_vx = list[i][3];
		double target_vy = list[i][4];
		double target_d = list[i][6];
		double target_s = list[i][5];

		double dist_s = target_s-egoPose.s;

		double v = sqrt(target_vx*target_vx + target_vy*target_vy);

		for (int lane=0; lane<3; lane++)
		{
			if (dist_s > 0 && 4*lane <= target_d && target_d <= 4*(lane+1))
			{
				// Don't forget to divide!
				lane_list[lane] += v;
				car_counter[lane]++;
			}
		}
	}

	for (int lane=0; lane<3; lane++)
		if (car_counter[lane] != 0)
			lane_list[lane] /= (double) car_counter[lane];
		else
			lane_list[lane] = 22.352; // v_max

	return lane_list;
}



trajectory_t StateMachine::evaluate_behavior(pose egoPose, targetList_t vehicle_list, int rest)
{
	state best_next_state;
	remaining_points = rest;
    unsigned int min_cost = UINT_MAX;
    
    std::vector<double> lane_speeds;

	/*std::random_device rd;
	std::normal_distribution<double> n_distrib_s(0., 15.);
	std::normal_distribution<double> n_distrib_d(0., .1);*/

    //std::map<state, double> costs;
    
    //std::cout << "Current state: " << current_state << std::endl;
    //for (auto iter : vehicle_list)
    //	cout << iter[0] << endl;

	// Frenet coordinates of ego-vehicle:
	if (rest == 0)
    {
    	std::vector<double> egoFrenet = maptool.getFrenet(egoPose.pos_x, egoPose.pos_y, egoPose.angle);
    	egoPose.s = egoFrenet[0];
    	egoPose.d = egoFrenet[1];
    	v_prev=0.;
    }
    else
    {
    	egoPose.s = s_prev;
    	egoPose.d = d_prev;
    }

    // Allocating space on stack for Trajectory object:
    Trajectory traj = Trajectory(egoPose, 0., 0., 0., 0., 1.);
    Trajectory best_next_trajectory = Trajectory(egoPose, 0., 0., 0., 0., 1.);

    // Checks whether the ego vehicle has arrived at the intended lane.
    if (abs(egoPose.d-4*intended_lane+2) < 5.) {
    	current_lane = intended_lane;
    	current_state = LK;
    }

	// Average speed per lane. If a lane is clear, then choose maximum speed (50mph).
    lane_speeds = lane_speeds_fct(egoPose, vehicle_list); 
    double delta_t;

    for (int i=0; i<3;i++) 
    	std::cout << "\tLane: " << i << ": " << lane_speeds[i] << "\t";
    std::cout << std::endl;


    for(auto state_iter : successor_states[current_state])
    {
		// Excluding the cases where the ego vehicle is on the left lane or the right land and considers lane change 
		// to nonexisting lanes.
	    if (!(state_iter == LCL && egoPose.d <= 4.) && !(state_iter == LCR && egoPose.d >= 8.))
	    {
	    	// One more for-loop here!
	    	for (int j=0; j<3; j++)
	    	{
			    double cost_for_state = 0;
			    double dd;
			    double ds;

    	    	switch(state_iter) {
		    		case LK:
		    			intended_lane = current_lane;
		    			break;
		    		case LCR:
		    			intended_lane = current_lane + 1;
		    			break;
		    		case LCL:
		    			intended_lane = current_lane - 1;
		    			break;
		    	}

		    	dd = (4*intended_lane + 2) - egoPose.d;
		    	ds = 90.;

			    // Generate new trajectory for state state_iter
			    //ds += n_distrib_s(rd);
			    //dd += n_distrib_d(rd);

				//traj = Trajectory(egoPose, ds, dd, 20., 20., time_const);
		    	delta_t = 2. * ds / (v_prev + lane_speeds[intended_lane]-1.);	// why doesn't this work?!?
		    	delta_t = floor(50. * delta_t) / 50.;
		    	
				traj = Trajectory(egoPose, ds, dd, v_prev, lane_speeds[intended_lane]-1, delta_t);

				// Add cost function for obstacles and road departure 
				cost_for_state += cost_function_traffic(traj, egoPose, vehicle_list, maptool);
				
				// Add cost function for trajectory profile (speed, jerk, etc.)
				cost_for_state += cost_function_behavior(traj, ds, dd);

				//costs[state_iter] = cost_for_state;

				if (cost_for_state <= min_cost)
				{
					best_next_state = state_iter;
					best_next_trajectory = traj;
					min_cost = cost_for_state;
				}
			}
		}
	}

	switch(best_next_state) {
		case LK:
			intended_lane = current_lane;
			break;
		case LCR:
			intended_lane = current_lane + 1;
			break;
		case LCL:
			intended_lane = current_lane - 1;
			break;
	}

    std::cout << "current_lane: " << current_lane << "\t | intended_lane: " << intended_lane << std::endl;

    delta_t = best_next_trajectory.getTime();
    std::cout << delta_t << std::endl;
	d_prev = best_next_trajectory.evaluate_d(delta_t);
	s_prev = best_next_trajectory.evaluate_s(delta_t);
	v_prev = lane_speeds[intended_lane]-1;

	return best_next_trajectory.getXY(maptool);
}


