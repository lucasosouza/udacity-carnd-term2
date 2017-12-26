#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


void NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  is_initialized_ = false;
  previous_timestamp_ = 0;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  // set number of dimensions in measurement
  n_z_radar_ = 3;
  n_z_laser_ = 2;

 ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights_
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // P_ <<    0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
  //         -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
  //          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
  //         -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
  //         -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  P_ << 10, 0, 0, 0, 0,
        0, 10, 0, 0, 0,
        0, 0, 10, 0, 0,
        0, 0, 0, 10, 0,
        0, 0, 0, 0, 10;

  // set measurement covariance matrix
  R_radar_ = MatrixXd(n_z_radar_,n_z_radar_);
  R_radar_ <<   std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0,std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(n_z_laser_,n_z_laser_);
  R_laser_ <<   std_laspx_*std_laspx_, 0,
                0, std_laspy_*std_laspy_;

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

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_ ) {
    /**
    TODO:
      * Initialize the state x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    if(fabs(meas_package.raw_measurements_[0]) < 0.0001 or fabs(meas_package.raw_measurements_[1]) < 0.0001) return;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /** Convert radar from polar to cartesian coordinates and initialize state. */
      float ro = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float ro_dot = meas_package.raw_measurements_[2];

      x_ << ro * cos(phi), ro * sin(phi), 0, 0, 0;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];

      x_ << px, py, 0, 0, 0;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    initial_timestamp_ = meas_package.timestamp_;
    previous_timestamp_ = initial_timestamp_;
    cout << "initialized" << endl;
    return;
  }

  if (meas_package.timestamp_ == initial_timestamp_) return;

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the timestamp_e elapsed between the current and previous measurements
  double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  // dealing with timesteps too large
  if (delta_t > 0.0001) {
    if (delta_t > 0.1) {
      while (delta_t > 0.1) {
        const double dt = 0.05;
        Prediction(dt);
        delta_t -= dt;
      }      
    } else {
      Prediction(delta_t);
    }
  }

  //original version:
  // if (delta_t > 0.0001) {
  //   Prediction(delta_t);
  // }

  // cout << "dt" << endl;
  // cout << dt << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_) {
    // Radar updates
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_){
    // Laser updates
    UpdateLidar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */

void UKF::Prediction(double delta_t){

  /* AUGMENTATION (include process noise)
      Augments X_sig with two additional dimensions related to process noise. Modifies input X_sig in place.
  */

  //create augmented mean vector
  VectorXd x_aug(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug(n_aug_, n_aug_);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma point matrix
  MatrixXd Xsig_aug(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }

  /* SIGMA POINT PREDICTION 
      Receives an empty matrix Xsig_out, modifies in place, and return void.
  */

  // cout << "Xsig_aug" << endl;
  // cout << Xsig_aug << endl;

  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;

    //angle normalization
    //reviewer: Normalizing the yaw of the predicted sigma points increases the RMSE values. Remove these lines
    //if (Xsig_pred_(3, i)> M_PI) Xsig_pred_(3, i) = fmod(Xsig_pred_(3, i),M_PI);
    //if (Xsig_pred_(3, i)<-M_PI) Xsig_pred_(3, i) = fmod(Xsig_pred_(3, i),M_PI);

  }

  // cout << "Xsig_pred_" << endl;
  // cout << Xsig_pred_ << endl;

  /* PREDICT MEAN (x) AND COVARIANCE (P) - posterior (x+1|x) 
    Updates x and P. Receives vector x and matrix P as inputs, modifies them in place, and return void.
  */
  
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);

  // alternative implementation , suggested by Oliver
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - Xsig_pred_.col(0);
    //angle normalization
    if (x_diff(3)> M_PI or x_diff(3)<-M_PI) {
      NormalizeAngle(x_diff(3));
    };

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

  // cout << "x_" << endl;
  // cout << x_ << endl;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  VectorXd z = meas_package.raw_measurements_;

  /* PREDICT LIDAR MEASUREMENT
      Converts CTRV model to lidar measuerements [px, py]. Receives as input two matrices, and modifies these matrices in place (return is void)

  */

  //create matrix for sigma points in measurement space
  MatrixXd Zsig(n_z_laser_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred(n_z_laser_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  MatrixXd S(n_z_laser_,n_z_laser_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_laser_;

  /* UPDATE STATE
      Updates x and P state, using as base the sigma points predicted for x+1 and projected in the measurement space. Modifies x and p in place and return void.
  */

  //calculate cross correlation matrix
  MatrixXd Tc(n_x_, n_z_laser_);
  Tc.fill(0.0);

  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - Zsig.col(0);

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    if (x_diff(3)> M_PI or x_diff(3)<-M_PI) {
      NormalizeAngle(x_diff(3));
    };

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S* K.transpose();

  // NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

  // cout << "NIS_laser_" << endl;
  // cout << NIS_laser_ << endl;

  // cout << "x_" << endl;
  // cout << x_ << endl;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.  

  You'll also need to calculate the radar NIS.
  */

  VectorXd z = meas_package.raw_measurements_;

  /* PREDICT RADAR MEASUREMENT
      Converts CTRV model to radar measuerements [rho, phi, rho_dot]. Receives as input two matrices, and modifies these matrices in place (return is void)

  */

  //create matrix for sigma points in measurement space
  MatrixXd Zsig(n_z_radar_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // check division by 0
    if (p_x == 0 && p_y == 0) {
      Zsig(0,i) = 0;
      Zsig(1,i) = 0;
      Zsig(2,i) = 0;
    } else {
      Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
      Zsig(1,i) = atan2(p_y, p_x);
      Zsig(2,i) = (p_x*v1 + p_y*v2)/sqrt(p_x*p_x + p_y*p_y);
    }

  }

  // cout << "Zsig" << endl;
  // cout << Zsig << endl;

  //mean predicted measurement
  VectorXd z_pred(n_z_radar_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  MatrixXd S(n_z_radar_,n_z_radar_);
  S.fill(0.0);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - Zsig.col(0);

    //angle normalization
    if (z_diff(1)> M_PI or z_diff(1)<-M_PI) {
      NormalizeAngle(z_diff(1));
    };

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_radar_;

  /* UPDATE STATE
      Updates x and P state, using as base the sigma points predicted for x+1 and projected in the measurement space. Modifies x and p in place and return void.
  */

  //calculate cross correlation matrix
  MatrixXd Tc(n_x_, n_z_radar_);
  Tc.fill(0.0);


  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - Zsig.col(0);
    if (z_diff(1)> M_PI or z_diff(1)<-M_PI) {
      NormalizeAngle(z_diff(1));
    };

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    if (x_diff(3)> M_PI or x_diff(3)<-M_PI) {
      NormalizeAngle(x_diff(3));
    };

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }


  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // cout << "z" << endl;
  // cout << z << endl;

  //residual
  VectorXd z_diff = z - z_pred;

  // cout << "z_pred" << endl;
  // cout << z_pred << endl;

  //angle normalization
  if (z_diff(1)> M_PI or z_diff(1)<-M_PI) {
    NormalizeAngle(z_diff(1));
  };

  // cout << "x_ before update" << endl;
  // cout << x_ << endl;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S* K.transpose();

  // cout << "x_ after update" << endl;
  // cout << x_ << endl;

  // NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

  // cout << "NIS_radar_" << endl;
  // cout << NIS_radar_ << endl;

};



