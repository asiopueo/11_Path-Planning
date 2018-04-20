#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <climits>
#include <map>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "common.h"







class StateMachine {

	public:
		StateMachine();
		~StateMachine();

		// method which evaluates behaviour
		void evaluate_behavior(pose, auto predictions);
		void execute_state_transition();
		vector<vector<double>> generate_trajectory(state, pose, auto vehicle_list);

	private:
		state current_state;
		state best_next_state;
		map<state, state> possible_successor_states;

		double weight_1, weight_2;

		//vector<vector<double>> generate_trajectory(state, pose, vehicle_list);
		
		double cost_function_1();
		double cost_function_2();
		double cost_function_3();
};




#endif /* STATEMACHINE_H */



