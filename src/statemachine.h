#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <climits>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


/*
 * STAY:  Stay on current lane
 * PLCL:  Prepare Lane Change Left
 * PLCR:  Prepare Lane Change Right
 * LCL:   Lane Change Left
 * LCR:   Lane Change Right
 */


enum state {STAY, PLCL, PLCR, LCL, LCR};


class StateMachine {

	public:
		StateMachine();
		~StateMachine();

		// method which evaluates behaviour
		void evaluate_situation(, );
		void execute_state_transition();

	private:
		state current_state;
		state best_next_state;
		map<vector<state>> possible_successor_states;

		vector<vector<double>> generate_trajectory(state, pose, vehicle_list);
		
		double cost_function_1();
		double cost_function_2();
		double cost_function_3();
};




#endif /* STATEMACHINE_H */



