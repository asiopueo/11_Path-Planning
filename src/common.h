struct pose {
	double pos_x;
	double pos_y;
	double angle;
	unsigned int lane;
};



void global2vehicle_coords(&)
{
	// Coordinate transformation (global->vehicle):
    Eigen::MatrixXd rotation = Eigen::MatrixXd(3,3);
    Eigen::MatrixXd translation = Eigen::MatrixXd(3,3);

    rotation << cos(angle), sin(angle), 0,
                sin(angle), cos(angle), 0,
                0, 0, 1;

    double t_x = 0.;
    double t_y = 0.;

    translation << 1, 0, -t_x,
                   0, 1, -t_y,
                   0, 0, 1;

    vector<double> X(3);
    vector<double> Y(3);

    for (int i=0; i<3; i++)
    {
      Eigen::VectorXd input(3), output(3);
      
      input << anchor_x_vals[i], anchor_y_vals[i], 1;
      output = rotation*translation*input;

      X.push_back(output(0));
      Y.push_back(output(1));
    }
}

void vehicle2global_coords(&)
{
	Eigen::VectorXd input(3), output(3);
	Eigen::MatrixXd rotation = Eigen::MatrixXd(3,3);
	Eigen::MatrixXd translation = Eigen::MatrixXd(3,3);

	rotation << cos(angle), -sin(angle), 0,
	          sin(angle), cos(angle), 0,
	          0, 0, 1;

	double t_x = 0.;
	double t_y = 0.;

	translation << 1, 0, t_x,
	             0, 1, t_y,
	             0, 0, 1;

	input << tmpX[i], tmpY[i], 1;
	output = rotation*translation*input;

	next_x_vals.push_back(output[0]);
	next_y_vals.push_back(output[1]);
}

