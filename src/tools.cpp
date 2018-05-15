#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	//Local variables
	VectorXd residual, rmse(4);

	//Initialize the rmse vector.
	rmse << 0, 0, 0, 0;

	// Check that the estimations size and ground truth size match before calculating the RMSE.
	if((estimations.size() != ground_truth.size()) || (estimations.size() == 0))
	{
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//Loop through and sum all the collected estimations(state vector)
	for(unsigned int i=0; i < estimations.size(); ++i)
	{
		// Element-wise subtraction of two 4x1 matricies (row x column) to calculate the difference.
	    residual = (estimations[i] - ground_truth[i]);
	    // Square of the difference
	    residual = residual.array().square();
	    // Add the squared difference to the current rmse value.
	    rmse += residual;
	}

	// Divide the squared difference value by the size of the estimation matrix.
	rmse = rmse/estimations.size();
	// Get the square root of the difference
	rmse = rmse.array().sqrt();

	return rmse;
}

double Tools::NormalizeAngle(double angle)
{
	//Constrain angle between -pi and pi.
	while ((angle > M_PI) || (angle < -M_PI))
	{
		if(angle > M_PI)
		{
			//cout << "Phi was adjusted between pi and -pi (" << angle << " > pi)." << endl;
			angle  -= 2*M_PI;
		}
		else if(angle < -M_PI)
		{
			//cout << "Phi was adjusted between pi and -pi (" << angle << " < -pi)." << endl;
			angle  += 2*M_PI;
		}
	}

	return angle;
}