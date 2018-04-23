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

		// method which evaluates behaviour
		std::vector<std::vector<double>> evaluate_behavior(pose ego_veh, std::vector<std::vector<double>> vehicle_list, int);
		void execute_state_transition();
		std::vector<std::vector<double>> generate_trajectory(state, pose, std::vector<std::vector<double>> vehicle_list);

	private:
		state current_state;
		state best_next_state;
		std::map<state, std::vector<state>> successor_states;
		int remaining_points;
		double dist_inc;

		unsigned int current_lane;
		unsigned int intended_lane;

		std::map<state, std::vector<std::vector<double>>> associated_trajectories;

		std::vector<double> weights;

		Maptool maptool;

		//vector<vector<double>> generate_trajectory(state, pose, vehicle_list);
		
		double cost_function_1(std::vector<std::vector<double>>, std::vector<std::vector<std::vector<double>>>);
};




#endif /* STATEMACHINE_H */



