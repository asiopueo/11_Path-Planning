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
		double d_prev; 
		double s_prev;
		double v_prev;

		int current_lane;
		int intended_lane;

		Maptool maptool;
		
		std::vector<double> weights;
		double cost_function_traffic(Trajectory &, const pose, const targetList_t, Maptool);
		double cost_function_behavior(const Trajectory &, double delta_s, double delta_d);
		std::vector<double> lane_speeds_fct(const pose egoPose, const targetList_t);
};




#endif /* STATEMACHINE_H */



