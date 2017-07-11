#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
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
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

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
  TODO:Done

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO: Done

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*****************************************************************************
   *  Initialization with first measurement data
   ****************************************************************************/
  if (!is_initialized_) {
    //Initialize x_, P_, previous_time, anything else needed.
    //Lesson7-32, need to tune v , yaw , yawd
    x_ << 1, 1, 0, 0, 0;

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      //if LASER data , Initialize here
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);

      x_ << px, py, 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      //if RADAR data , Initialize here
      double ro = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      // float ro_dot = meas_package.raw_measurements_(2); // not used

      double px = ro*cos(phi);
      double py = ro*sin(phi);

      x_ << px, py, 0, 0, 0;
    }

    float std_i = 1;
    P_ << std_i, 0., 0., 0., 0.,
          0., std_i, 0., 0., 0.,
          0., 0., std_i, 0., 0.,
          0., 0., 0., std_i, 0.,
          0., 0., 0., 0., std_i;

    // Init. previous time
    time_us_ = meas_package.timestamp_;

    is_initialized_ = true;
    return;
  } // end of initialize

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; // delta_t - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
    UpdateLidar(meas_package);
  } else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
    UpdateRadar(meas_package);
  }

}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:Done

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /*****************************************************************************
   *  Generate Sigma Points
   ****************************************************************************/

  //create augmented state vector
  // VectorXd x_aug = VectorXd(7);
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  // MatrixXd P_aug = MatrixXd(7, 7);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();

  //Q matrix
  MatrixXd Q(2, 2);
  Q << std_a_*std_a_, 0,
        0, std_yawdd_*std_yawdd_;

  P_aug.topLeftCorner<5, 5>() = P_;
  P_aug.bottomRightCorner<2, 2>() = Q;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0)  = x_aug;

  //create augmented sigma points
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  /*****************************************************************************
   *  Calculate predicted state vector 'x_pred' from augmented sigma Points
   ****************************************************************************/

   //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = px + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = py + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_p = px + v*delta_t*cos(yaw);
      py_p = py + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // convenience for following calculation
    double const delta_t2 = delta_t*delta_t;

    // add noise
    px_p += 0.5*nu_a*delta_t2*cos(yaw);
    py_p += 0.5*nu_a*delta_t2*sin(yaw);
    v_p += nu_a*delta_t;
    yaw_p += 0.5*nu_yawdd*delta_t2;
    yawd_p += nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  /*****************************************************************************
   *  Calculate predicted Mean and Covariance
   ****************************************************************************/

  // set weights_
  weights_.fill(0.5/(n_aug_+lambda_));
  weights_(0) = lambda_/(lambda_+n_aug_);

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization , yaw
    x_diff(3) = tools.NormalizeAng(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO: Done

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // Get real measurement
  VectorXd z = meas_package.raw_measurements_;

  // From section 10 in lesson 5
  // extract only px,py in predicted state
  MatrixXd H_laser = MatrixXd(2, 5);
  H_laser << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0;

  //measurement noise matrix - laser
  // Look through manufacturer's manul for this value
  MatrixXd R_laser = MatrixXd(2, 2);
  R_laser << std_laspx_*std_laspx_, 0,
             0, std_laspy_*std_laspy_;

  // In section 7 of Lesson 5 , calc. Kalman gain K
  VectorXd z_pred = H_laser * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_laser.transpose();
  MatrixXd S = H_laser * P_ * Ht + R_laser;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // Update new state x_ , P_
  x_ += (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser) * P_;

  //Calculate the lidar NIS.
  // double NIS_laser = y.transpose() * S.inverse() * y;
  // std::cout << "NIS Laser = " << NIS_laser << std::endl;
  //
  // std::cout << "x = \n" << x_ << std::endl;
  // std::cout << "P = \n" << P_ << std::endl;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:Done

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //set measurement dimension, radar can measure rho, phi, and rho_dot
  int n_z = 3;

  /*****************************************************************************
   *  Predict Radar Measurement
   ****************************************************************************/
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double px  = Xsig_pred_(0,i);
    double py  = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double vx = cos(yaw)*v;
    double vy = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(px*px + py*py);                       //rho
    Zsig(1,i) = atan2(py,px);                             //phi
    Zsig(2,i) = (px*vx + py*vy ) / sqrt(px*px + py*py);   //rho_dot
  }

  //mean of predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  // lambda_ = -1.0;
  // weights_.fill(0.5/(lambda_+ n_z));
  // weights_.(0) = lambda_/(lambda_+ n_z);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred += weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization , phi
    z_diff(1) = tools.NormalizeAng(z_diff(1));

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S += R;

  /*****************************************************************************
   *  UpdateState
   ****************************************************************************/
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = tools.NormalizeAng(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    x_diff(3) = tools.NormalizeAng(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = tools.NormalizeAng(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // double NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
  //std::cout << "NIS Radar= " << NIS_radar << std::endl;

  // std::cout << "x = \n" << x_ << std::endl;
  // std::cout << "P = \n" << P_ << std::endl;
}
