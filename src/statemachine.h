#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <climits>
#include <map>
#include <vector>
#include "maptool.h"
#include "common.h"





/*
 * STAY:  Stay on current lane
 * PLCL:  Prepare Lane Change Left
 * PLCR:  Prepare Lane Change Right
 * LCL:   Lane Change Left
 * LCR:   Lane Change Right
 */

enum state {LK, PLCL, PLCR, LCL, LCR, EA};



class StateMachine {

	public:
		StateMachine();
		~StateMachine();

		// Method which evaluates behaviour
		trajectory_t evaluate_behavior(pose ego_veh, targetList_t vehicle_list, int);
		
	private:
		state current_state;
		state best_next_state;
		std::map<state, std::vector<state>> successor_states;
		int remaining_points;
		double dist_inc;

		unsigned int current_lane;
		unsigned int intended_lane;

		std::map<state, trajectory_t> associated_trajectories;

		Maptool maptool;
		
		trajectory_t generate_trajectory(state, pose);

		std::vector<double> weights;
		double cost_function_0(trajectory_t, pose, targetList_t);
		double cost_function_1(trajectory_t);
};




#endif /* STATEMACHINE_H */



