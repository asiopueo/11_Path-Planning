#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <climits>
#include <map>
#include "common.h"
#include "maptool.h"




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
		std::vector<std::vector<double>> evaluate_behavior(pose ego_veh, vector<vector<double>> vehicle_list, int);
		void execute_state_transition();
		std::vector<std::vector<double>> generate_trajectory(state, pose, std::vector<std::vector<double>> vehicle_list);

	private:
		state current_state;
		state best_next_state;
		std::map<state, state> successor_states;
		int remaining_points;
		double dist_inc;

		unsigned int current_lane;
		unsigned int intended_lane;

		std::map<state, vector<vector<double>>> associated_trajectories;

		vector<double> weights;

		Maptool maptool;

		//vector<vector<double>> generate_trajectory(state, pose, vehicle_list);
		
		double cost_function_1(vector<vector<double>>, vector<vector<double>>);
		double cost_function_2(vector<vector<double>>, vector<vector<double>>);
};




#endif /* STATEMACHINE_H */



