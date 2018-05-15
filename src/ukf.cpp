#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;//Original is 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3; //Original is 30
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //Set the initialized state, so the first measurement is used to initialize the state vector.
  is_initialized_ = false;

  //Initialize timestamp variables
  previous_timestamp_ = 0;

  //Initialize the nis value
  nis_ = 0 ;

  //The number of dimensions our state has [px, py, velocity, yaw angle, yaw rate]
  state_dimension_ = 5;

  //Augments the state vector with 2 additional parameters [vak, vpk]
  state_dimension_augmented_ = 7;

  //Number sigma points needed for the model.
  n_sigma_points_augmented_ = 2*(state_dimension_augmented_) + 1;

  //Create a sigma point matrix
  X_sigma_points_predicted_ = MatrixXd(state_dimension_, n_sigma_points_augmented_);
  
  //design parameters
  lambda_ = 3 - state_dimension_augmented_;

  //Initialize the weights
  weights_ = VectorXd(n_sigma_points_augmented_);
  //Store the first weight
  weights_(0) = lambda_/(lambda_+state_dimension_augmented_);
  //Store the [2, 3, 4, ..., n_sigma_points_augmented] weights
  for(int i = 1; i < n_sigma_points_augmented_; i++)
  {
      weights_(i) = 1/(2*(lambda_+state_dimension_augmented_));
  }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  float delta_t;
  time_us_ = meas_package.timestamp_;
  delta_t = float(time_us_ - previous_timestamp_) / 1000000.0;

  if (!is_initialized_) 
  {
    // Initialization
    // Changed from 1 to 0.1 (improved the vy RMSE value)
    P_ << 0.1,0,0,0,0,
          0,0.1,0,0,0,
          0,0,0.1,0,0,
          0,0,0,0.1,0,
          0,0,0,0,0.1;

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho, theta, rho_dot;
      rho = meas_package.raw_measurements_[0];
      theta = meas_package.raw_measurements_[1];
      rho_dot = meas_package.raw_measurements_[2];
      
      x_(0) = rho*cos(theta);
      x_(1) = rho*sin(theta);
      x_(2) = sqrt(pow(rho_dot*cos(theta),2)+pow(rho_dot*sin(theta),2));
      x_(3) = 0;
      x_(4) = 0;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      double px, py;
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];

      x_(0) = px;
      x_(1) = py;
      // Changed from 0 to 1 (improved the vx and vy RMSE value)
      x_(2) = 1;
      x_(3) = 1;
      x_(4) = 1;
    }

    is_initialized_ = true;
  }
  else
  {
    //
    Prediction(delta_t);


    if((meas_package.sensor_type_ == MeasurementPackage::RADAR && (use_radar_ == true)))
    {
      //Update with radar measurements
      nis_ = UpdateRadar(meas_package);
    }
    else if((meas_package.sensor_type_ == MeasurementPackage::LASER && (use_laser_ == true)))
    {
      //Update with lidar measurements
      nis_ = UpdateLidar(meas_package);
    }
    else
    {
      //Error
      cout << "Error - Unknown Sensor" << endl;
    }
  }
  
  previous_timestamp_ = time_us_;

}

/**
 * Returns the NIS value calculated from the current sensor data.
 */
double UKF::GetNIS() {

  return nis_;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //double px, py;
  double velocity, yaw_angle, yaw_rate;
  double nu_a, nu_p;
  
  MatrixXd sub_matrix;
  MatrixXd P_sqrt;
  VectorXd x_difference;
  VectorXd x_delta;
  VectorXd nu_k;

  //Calligraphic-X for sigma points
  MatrixXd X_sigma_points_augmented;
  
  //Stores 2 additional parameters
  VectorXd x_augmented;
  //Stpres 2 additional parameters
  MatrixXd P_augmented;

  //Generate sigma points section
  /*-----------------------------------------------------------------------*/

  x_augmented = VectorXd(state_dimension_augmented_);
  P_augmented = MatrixXd(state_dimension_augmented_, state_dimension_augmented_);
  X_sigma_points_augmented = MatrixXd(state_dimension_augmented_, n_sigma_points_augmented_); 

  //Augment the state vector with nu longitudinal acceleration and nu yaw acceleration
  x_augmented.head(state_dimension_) = x_;
  x_augmented(state_dimension_)      = 0;
  x_augmented(state_dimension_+1)    = 0;

  //Augment the uncertainty covariance matrix with the process noise covariance matrix.
  //Fill the matrix with zeros.
  P_augmented.fill(0.0);
  P_augmented.topLeftCorner(state_dimension_, state_dimension_) = P_;
  //Add the process noise covariance matrix
  P_augmented(state_dimension_, state_dimension_)     = pow(std_a_, 2);
  P_augmented(state_dimension_+1, state_dimension_+1) = pow(std_yawdd_, 2);
  
  //Calculate the sqareroot of P.
  P_sqrt = P_augmented.llt().matrixL();

  //Calculate the sub matrix.
  sub_matrix = sqrt(lambda_+state_dimension_augmented_)*P_sqrt;

  //Create the sigma point matrix.
  //Store the first sigma point.
  X_sigma_points_augmented.col(0) = x_augmented;
  //Store the other sigma points.
  for(int i = 0; i < state_dimension_augmented_; i++)
  {
    X_sigma_points_augmented.col(i+1)                            = (x_augmented + sub_matrix.col(i));
    X_sigma_points_augmented.col(i+1+state_dimension_augmented_) = (x_augmented - sub_matrix.col(i));
  }

  //Predict Sigma Points
  /*-----------------------------------------------------------------------*/

  //Initialize variables
  nu_k = VectorXd(state_dimension_);
  x_delta = VectorXd(5);

  for(int i = 0; i < n_sigma_points_augmented_; i++)
  {
    //Copy the augmented state vector to readable variables for the following calculations.
    //px        = X_sigma_points_augmented.col(i)(0); // position x at time k
    //py        = X_sigma_points_augmented.col(i)(1); // position y at time k
    velocity  = X_sigma_points_augmented.col(i)(2); // Velocity at time k
    yaw_angle = X_sigma_points_augmented.col(i)(3); // yaw angle at time k
    yaw_rate  = X_sigma_points_augmented.col(i)(4); // yaw rate (derivative of the yaw angle) at time k
    nu_a      = X_sigma_points_augmented.col(i)(5); // Standard deviations longitudinal acceleration
    nu_p      = X_sigma_points_augmented.col(i)(6); // Standard deviations of yaw acceleration

    if(fabs(yaw_rate) > 0.001)
    {
      //When the yaw rate is not zero.
      x_delta(0) = (velocity/yaw_rate) * ( sin(yaw_angle + yaw_rate * delta_t) - sin(yaw_angle));
      x_delta(1) = (velocity/yaw_rate) * (-cos(yaw_angle + yaw_rate * delta_t) + cos(yaw_angle));
    }
    else
    {
      //When the yaw rate is zero.
      x_delta(0) = velocity * cos(yaw_angle) * delta_t;
      x_delta(1) = velocity * sin(yaw_angle) * delta_t;
    }

    x_delta(2) = 0;
    x_delta(3) = yaw_rate * delta_t;
    x_delta(4) = 0;
    
    //Noise vector nu at time k (vk)
    nu_k << (0.5 * pow(delta_t, 2)) * (cos(yaw_angle) * nu_a), //Influence of longitudinal acceleration on position x (yaw acceleration is negligible, so it is not considered)
            (0.5 * pow(delta_t, 2)) * (sin(yaw_angle) * nu_a), //Influence of longitudinal acceleration on position y (yaw acceleration is negligible, so it is not considered)
            delta_t * nu_a,                                    //Influence of yaw acceleration on velocity
            0.5 * pow(delta_t, 2) * nu_p,                      //Influence of yaw acceleration on yaw angle
            delta_t * nu_p;                                    //Influence of yaw acceleration on yaw rate

    //x_k+1 = x_k + x_delta + nu_k;
    X_sigma_points_predicted_.col(i).head(state_dimension_) = X_sigma_points_augmented.col(i).head(state_dimension_) + x_delta + nu_k;
  }

  // Predict state mean and covariance
  /*-----------------------------------------------------------------------*/
  // Predict state mean
  x_.fill(0.0);
  for(int i = 0; i < n_sigma_points_augmented_; i++)
  {
    //x_k+1|k = sum(w_i*X_k+1|k,i) 
    x_ += weights_(i)*X_sigma_points_predicted_.col(i);
    //cout << X_sigma_points_predicted_.col(i) << ":" << weights_(i) << endl;
  }

  // Predict state covariance matrix
  P_.fill(0.0);
  for(int i = 0; i < n_sigma_points_augmented_; i++)
  {
    x_difference = X_sigma_points_predicted_.col(i) - x_;

    //Make sure x_difference(3) is between -pi and pi as indicated in the Udacity lession.
    x_difference(3) = tools_.NormalizeAngle(x_difference(3));

    P_ += weights_(i)*x_difference*x_difference.transpose();
    //cout << x_difference*x_difference.transpose() << endl;
  }

}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
double UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  double px, py;
  double nis_lidar;
  MatrixXd Z_sigma_points = MatrixXd(2, n_sigma_points_augmented_);
  VectorXd z_prediction = VectorXd(2);
  VectorXd z_measurement = VectorXd(2);
  VectorXd z_difference, x_difference;
  MatrixXd K;
  MatrixXd S = MatrixXd(2, 2);
  MatrixXd R = MatrixXd(2, 2);
  MatrixXd Tc = MatrixXd(5, 2);

  // Predict measurement and update state
  /*-----------------------------------------------------------------------*/
  nis_lidar = 0;
  z_prediction.fill(0.0);
  R.fill(0.0);
  S.fill(0.0);
  Tc.fill(0.0);

  z_measurement = meas_package.raw_measurements_.head(2);

  for(int i = 0; i < n_sigma_points_augmented_; i++)
  {
      px = X_sigma_points_predicted_(0, i);
      py = X_sigma_points_predicted_(1, i);
      
      //z is the measurement vector
      Z_sigma_points(0, i) = px;
      Z_sigma_points(1, i) = py;
      
      z_prediction += weights_(i)*Z_sigma_points.col(i);
  }

  R(0, 0) = pow(std_laspx_, 2);
  R(1, 1) = pow(std_laspy_, 2);

  for(int i = 0; i < n_sigma_points_augmented_; i++)
  {
    //Calculate the differences
    z_difference = Z_sigma_points.col(i) - z_prediction;
    x_difference = X_sigma_points_predicted_.col(i) - x_;

    //Make sure z_difference(1) is between -pi and pi as indicated in the Udacity lession.
    //Normalize angle
    x_difference(3) = tools_.NormalizeAngle(x_difference(3));

    S += weights_(i) * z_difference * z_difference.transpose();
    //cout << z_difference * z_difference.transpose() << endl;

    //cross correlation matrix
    Tc += weights_(i) * x_difference * z_difference.transpose();
  }

  //Add measurement noise covariance matrix
  S+=R;
  
  K = Tc*S.inverse();

  z_difference = (z_measurement - z_prediction);
  
  x_ += K*(z_difference);
  
  P_ += -K*S*K.transpose();

  //NIS (e ~ X^2 (Xi squared distribution))
  nis_lidar = z_difference.transpose() * S.inverse() * z_difference;
  //cout << "LIDAR_NIS: " << nis_lidar << endl;

  return nis_lidar;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
double UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  double px, py, velocity, yaw_angle;
  double nis_radar;
  MatrixXd Z_sigma_points = MatrixXd(3, n_sigma_points_augmented_);
  VectorXd z_prediction = VectorXd(3);
  VectorXd z_measurement = VectorXd(3);
  VectorXd z_difference, x_difference;
  MatrixXd K;
  MatrixXd S = MatrixXd(3, 3);
  MatrixXd R = MatrixXd(3, 3);
  MatrixXd Tc = MatrixXd(5, 3);

  // Predict measurement and update state
  /*-----------------------------------------------------------------------*/
  nis_radar = 0;
  z_prediction.fill(0.0);
  R.fill(0.0);
  S.fill(0.0);
  Tc.fill(0.0);

  z_measurement = meas_package.raw_measurements_.head(3);
  
  for(int i = 0; i < n_sigma_points_augmented_; i++)
  {
      px        = X_sigma_points_predicted_(0, i);
      py        = X_sigma_points_predicted_(1, i);
      velocity  = X_sigma_points_predicted_(2, i);
      yaw_angle = X_sigma_points_predicted_(3, i);
      
      //z is the measurement vector
      Z_sigma_points(0, i) = sqrt(pow(px, 2) + pow(py, 2));
      Z_sigma_points(1, i) = atan2(py, px);;
      Z_sigma_points(2, i) = velocity*(px*cos(yaw_angle) + py*sin(yaw_angle))/Z_sigma_points(0, i);
      
      z_prediction += weights_(i)*Z_sigma_points.col(i);
  }

  //Measurement noise covariance
  R(0, 0) = pow(std_radr_, 2);
  R(1, 1) = pow(std_radphi_, 2);
  R(2, 2) = pow(std_radrd_, 2);


  for(int i = 0; i < n_sigma_points_augmented_; i++)
  {
    //Calculate the differences
    z_difference = Z_sigma_points.col(i) - z_prediction;
    x_difference = X_sigma_points_predicted_.col(i) - x_;

    //Make sure z_difference(1) is between -pi and pi as indicated in the Udacity lession.
    //Normalize angle
    z_difference(1) = tools_.NormalizeAngle(z_difference(1));
    x_difference(3) = tools_.NormalizeAngle(x_difference(3));

    S += weights_(i) * z_difference * z_difference.transpose();
    //cout << z_difference * z_difference.transpose() << endl;

    //cross correlation matrix
    Tc += weights_(i) * x_difference * z_difference.transpose();
  }

  //Add measurement noise covariance matrix
  S+=R;

  K = Tc*S.inverse();

  z_difference = (z_measurement - z_prediction);
  
  x_ += K*(z_difference);
  
  P_ += -K*S*K.transpose();

  //NIS (e ~ X^2 (Xi squared distribution))
  nis_radar = z_difference.transpose() * S.inverse() * z_difference;
  //cout << "RADAR_NIS: " << nis_radar << endl;
  return nis_radar;
}
