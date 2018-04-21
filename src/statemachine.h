#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <climits>
#include <map>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "common.h"
#include "maptool.h"






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

		double weight_1, weight_2;

		Maptool maptool;

		//vector<vector<double>> generate_trajectory(state, pose, vehicle_list);
		
		double cost_function_1(vector<vector<double>>);
		double cost_function_2(vector<vector<double>>);
};




#endif /* STATEMACHINE_H */



