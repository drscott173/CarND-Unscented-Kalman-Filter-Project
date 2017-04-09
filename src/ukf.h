#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
 private:

  long long previous_timestamp_;
  void InitMeasurement(MeasurementPackage meas_package);
  void ProcessRadarMeasurement(MeasurementPackage meas_package, float dt);
  void ProcessLaserMeasurement(MeasurementPackage meas_package, float dt);
  void GenerateAugmentedSigmaPoints();
  void PredictSigmaPoints(double delta_t);
  void PredictMeanAndCovariance();
  bool PredictRadarMeasurement();
  void UpdateStateWithRadar(VectorXd z);
  void PredictLaserMeasurement();
  void UpdateStateWithLaser(VectorXd z);
  void UpdateStateCommon(VectorXd z,      // actual measurement
			 VectorXd z_pred, // predicted measurement
			 MatrixXd Zsig,   // predicted sigma points
			 MatrixXd &Tc,     // Cross correlation matrix
			 MatrixXd S,      // Measurement covariance (uncertainty)
			 MatrixXd R);    // Measurement noise covariance (uncertainty)
  

 public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* augmented state [pos1 pos2 vel_abs yaw_angle yaw_rate xxx xxx] in SI units and rad
  VectorXd x_aug_;

  ///* augmented sigma points
  MatrixXd Xsig_aug_;

  ///* augmented sigma points, in measurement space
  MatrixXd Zsig_radar_;
  MatrixXd Zsig_laser_;

  ///* predicted measurement points
  VectorXd z_pred_radar_;
  VectorXd z_pred_laser_;

  ///* measurement covariance matrix S
  MatrixXd S_radar_;
  MatrixXd S_laser_;

  ///* measurement noise covariance matrix S
  MatrixXd R_radar_;
  MatrixXd R_laser_;

  ///* cross correlation matrix Tc
  MatrixXd Tc_radar_;
  MatrixXd Tc_laser_;

  ///* augmented state covariance matrix
  MatrixXd P_aug_;

  ///* square root of augmented covariance matrix
  MatrixXd L_;

  ///* Predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Measurement dimension
  int n_z_radar_;
  int n_z_laser_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();



  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Inititialize all variables
   */
  void Setup(double a_straight, double a_turn);

};

#endif /* UKF_H */
