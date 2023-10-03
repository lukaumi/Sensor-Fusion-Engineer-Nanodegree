#include <iostream>

#include "ukf.h"
#include "Eigen/Dense"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = Eigen::VectorXd(5);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // orig. 30
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // orig. 30
  std_yawdd_ = 1.0;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values
   */

  /**
   * TODO: Complete the initialization.
   * See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   * DONE
   */

  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // predicted sigma points matrix
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  // time when the state is true, in us
  time_us_ = 0.0;

  // Weights of sigma points
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function!
   * Make sure you switch between lidar and radar measurements.
   * DONE
   */

  if (!is_initialized_)
  {
    // set the state with the initial location and zero velocity
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER)
    {
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1], 0, 0, 0;

      P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
          0, std_laspy_ * std_laspy_, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR)
    {
      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      const double rho_dot = meas_package.raw_measurements_(2);

      // const double vx = rho_dot * cos(phi);
      // const double vy = rho_dot * sin(phi);
      // const double v = std::sqrt(vx * vx + vy * vy);

      x_ << rho * cos(phi), // x
          rho * sin(phi),   // y
          rho_dot,          // v
          rho,
          rho_dot;

      P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
          0, std_radr_ * std_radr_, 0, 0, 0,
          0, 0, std_radrd_ * std_radrd_, 0, 0,
          0, 0, 0, std_radphi_ * std_radphi_, 0,
          0, 0, 0, 0, std_radphi_ * std_radphi_;
    }
    else
    {
      std::cout << "ERROR: Invalid measurement sensor type " << meas_package.sensor_type_ << std::endl;
      exit(EXIT_FAILURE);
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
  }
  else
  {
    // compute the time elapsed between the current and previous measurements (in seconds)
    const double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;

    // update overall time
    time_us_ = meas_package.timestamp_;

    // Predict the next states and covariance matrix
    Prediction(dt);

    // Update the next states and covariance matrix
    switch (meas_package.sensor_type_)
    {
    case MeasurementPackage::SensorType::LASER:
      UpdateLidar(meas_package);
      break;
    case MeasurementPackage::SensorType::RADAR:
      UpdateRadar(meas_package);
      break;
    default:
      std::cout << "ERROR: Invalid measurement sensor type " << meas_package.sensor_type_ << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

void UKF::Prediction(double delta_t)
{
  /**
   * TODO: Complete this function!
   * Estimate the object's location.
   * Modify the state vector, x_.
   * Predict sigma points, the state, and the state covariance matrix.
   * DONE
   */

  // generate augmented sigma points

  // create augmented mean vector
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(5) = x_;

  // create augmented state covariance
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug.bottomRightCorner(2, 2) << std_a_ * std_a_, 0, 0, std_yawdd_ * std_yawdd_;

  // create square root matrix
  Eigen::MatrixXd L = P_aug.llt().matrixL();

  // create sigma point matrix
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // predict sigma points and add to matrix

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    // extract state vector elements
    const double px = Xsig_aug(0, i);
    const double py = Xsig_aug(1, i);
    const double v = Xsig_aug(2, i);
    const double yaw = Xsig_aug(3, i);
    const double yaw_rate = Xsig_aug(4, i);
    const double nu_accel = Xsig_aug(5, i);
    const double nu_yawdd = Xsig_aug(6, i);

    // predict the states
    double px_pred;
    double py_pred;

    // avoid division by zero
    if (std::abs(yaw_rate) > 0.001)
    {
      px_pred = px + v / yaw_rate * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
      py_pred = py + v / yaw_rate * (-1 * cos(yaw + yaw_rate * delta_t) + cos(yaw));
    }
    else
    {
      px_pred = px + v * cos(yaw) * delta_t;
      py_pred = py + v * sin(yaw) * delta_t;
    }

    double v_pred = v;
    double yaw_pred = yaw + yaw_rate * delta_t;
    double yawd_pred = yaw_rate;

    // add noise
    px_pred += 0.5 * delta_t * delta_t * cos(yaw) * nu_accel;
    py_pred += 0.5 * delta_t * delta_t * sin(yaw) * nu_accel;
    v_pred += delta_t * nu_accel;
    yaw_pred += 0.5 * delta_t * delta_t * nu_yawdd;
    yawd_pred += delta_t * nu_yawdd;

    // write predicted sigma points into right column
    Xsig_pred_(0, i) = px_pred;
    Xsig_pred_(1, i) = py_pred;
    Xsig_pred_(2, i) = v_pred;
    Xsig_pred_(3, i) = yaw_pred;
    Xsig_pred_(4, i) = yawd_pred;
  }

  // predict mean and covariance

  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    // state difference
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // normalize the yaw angle within -pi to + pi
    while (x_diff(3) > M_PI)
    {
      x_diff(3) -= 2.0 * M_PI;
    }
    while (x_diff(3) < -M_PI)
    {
      x_diff(3) += 2.0 * M_PI;
    }

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function!
   * Use lidar data to update the belief about the object's position.
   * Modify the state vector, x_, and covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   * DONE
   */

  // lidar noise profile is linear, so we can simply
  // use the standard linear Kalman filter

  // measurement dimension, lidar can measure x and y
  const uint8_t n_z = 2;

  // measurement matrix
  Eigen::MatrixXd H = Eigen::MatrixXd(n_z, n_x_);
  H.fill(0.0);
  H(0, 0) = 1;
  H(1, 1) = 1;

  // measurements
  Eigen::VectorXd z_pred = H * x_;
  const Eigen::VectorXd z = meas_package.raw_measurements_;

  // calculate residual vector y
  const Eigen::VectorXd y = z - z_pred;

  // measurement noise matrix
  R_lidar_.setZero(n_z, n_z);
  R_lidar_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

  // innovation covariance matrix S
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R_lidar_;

  // Kalman gain matrix K
  const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  // create new estimate for states and covariance
  x_ = x_ + (K * y);
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H) * P_;

  // calculate NIS for lidar
  NIS_lidar_ = y.transpose() * S.inverse() * y;

  // std::cout << "NIS lidar: " << NIS_lidar_ << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function!
   * Use radar data to update the belief about the object's position.
   * Modify the state vector, x_, and covariance, P_.
   * You can also calculate the radar NIS, if desired.
   * DONE
   */

  // measurement dimension, radar can measure r, phi, and r_dot
  const uint8_t n_z = 3;

  // transform sigma points in measurement space

  Eigen::MatrixXd Zsig = Eigen::MatrixXd(3, 2 * n_aug_ + 1);

  Zsig.row(0) = sqrt(
      (Xsig_pred_.row(0).array() * Xsig_pred_.row(0).array()) +
      (Xsig_pred_.row(1).array() * Xsig_pred_.row(1).array()));

  for (int i = 0; i < Zsig.cols(); i++)
  {
    Zsig(1, i) = std::atan2(Xsig_pred_(1, i), Xsig_pred_(0, i));
  }

  Zsig.row(2) = ((Xsig_pred_.row(0).array() * Xsig_pred_.row(2).array() * Xsig_pred_.row(3).array().cos()) +
                 (Xsig_pred_.row(1).array() * Xsig_pred_.row(2).array() * Xsig_pred_.row(3).array().sin())) /
                Zsig.row(0).array();

  // mean predicted measurement
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
  z_pred = Zsig * weights_;

  // measurement covariance matrix R
  R_radar_.setZero(n_z, n_z);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;

  // calculate innovation covariance matrix S
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S = (Zsig.array().colwise() - z_pred.array());
  S = S * weights_.asDiagonal() * S.transpose() + R_radar_;

  // cross correlation matrix Tc
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);
  Tc = (Xsig_pred_.array().colwise() - x_.array()).matrix() *
       weights_.asDiagonal() *
       (Zsig.array().colwise() - z_pred.array()).matrix().transpose();

  // Kalman gain K
  const Eigen::MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  const Eigen::VectorXd z = meas_package.raw_measurements_;
  const Eigen::VectorXd y = z - z_pred;
  x_ = x_ + (K * y);
  P_ = P_ - K * S * K.transpose();

  // calculate NIS for radar
  NIS_radar_ = y.transpose() * S.inverse() * y;

  // std::cout << "NIS radar: " << NIS_radar_ << std::endl;
}
