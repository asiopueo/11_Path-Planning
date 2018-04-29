#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <climits>
#include <map>
#include <vector>
//#include "maptool.h"
#include "common.h"
#include "trajectory.h"
#include <cmath>
#include <iostream>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// Only necessary when fooling around with splines:
// #include "spline.h"





class StateMachine {

	public:
		StateMachine();
		~StateMachine();

		// Method which evaluates behaviour
		trajectory_t evaluate_behavior(pose egoPose, targetList_t vehicle_list, int);
		
	private:
		state current_state;
		state best_next_state;
		std::map<state, std::vector<state>> successor_states;
		int remaining_points;

		unsigned int current_lane;
		unsigned int intended_lane;

		Maptool maptool;
		
		std::vector<double> weights;
		double cost_function_0(Trajectory &, const pose, const targetList_t, Maptool);
		double cost_function_1(const Trajectory &, double delta_s, double delta_d);
};




#endif /* STATEMACHINE_H */



