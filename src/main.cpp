
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // laser measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // radar measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

      // read ground truth data to compare later
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      gt_package.gt_values_ = VectorXd(4);
      gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
      gt_pack_list.push_back(gt_package);
  }

  // Create a UKF instance
  UKF ukf;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // start filtering from the second frame (the speed is unknown in the first
  // frame)

  size_t number_of_measurements = measurement_pack_list.size();

  // column names for output file
  out_file_ << "px" << "\t";
  out_file_ << "py" << "\t";
  out_file_ << "v" << "\t";
  out_file_ << "yaw_angle" << "\t";
  out_file_ << "yaw_rate" << "\t";
  out_file_ << "px_measured" << "\t";
  out_file_ << "py_measured" << "\t";
  out_file_ << "px_true" << "\t";
  out_file_ << "py_true" << "\t";
  out_file_ << "vx_true" << "\t";
  out_file_ << "vy_true" << "\t";
  out_file_ << "NIS" << "\n";


  Tools tools;
  double best_score = 1e9, best_i=0.0, best_j=0.0;

  // was 2.8, 3.2

  if (true) {
    for (int i=0; i < 200; i++) {
      cout << "**Loop " << i << endl;
      for (int j=0; j < 200; j++) {

	ukf.Setup(i*0.02, j*0.02);
	  //	ukf.Setup(2.7+i*0.01, 3.1+j*0.01);
	
	for (size_t k = 0; k < number_of_measurements; ++k) {
	  
	  // Call the UKF-based fusion
	  ukf.ProcessMeasurement(measurement_pack_list[k]);
	  
	  // convert ukf x vector to cartesian to compare to ground truth
	  VectorXd ukf_x_cartesian_ = VectorXd(4);
	  
	  float x_estimate_ = ukf.x_(0);
	  float y_estimate_ = ukf.x_(1);
	  float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
	  float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
	  
	  ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
	  estimations.push_back(ukf_x_cartesian_);
	  ground_truth.push_back(gt_pack_list[k].gt_values_);
	}
	VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
	double score = 0.25*(rmse[0]*100.0 + rmse[1]*100.0 + rmse[2] + rmse[3]);
	
	if (score < best_score) {
	  best_score = score;
	  cout << "Score now " << score << endl;
	  best_i = i;
	  best_j = j;
	  //	  cout << "Using accel straight " << (2.7+i*0.01) << " turn " << (3.1+j*0.01) << endl;
	  cout << "Using accel straight " << (0.02*i) << " turn " << (0.02*j) << endl;
	}
	
	estimations.clear();
	ground_truth.clear();
      }
    }
  }

  cout << "**Doing main loop**" << endl;

  // ukf.Setup(0.56, 0.02); // for second data set
  //  ukf.Setup(2.8, 3.2);

  //  2.8, 3.2 in scenario 1
  // 0.56, 0.02 in scenario 2
  ukf.Setup(0.02*best_i, 0.02*best_j);

  //  ukf.Setup(2.8, 3.2);
  //  ukf.Setup(0.8, 0.8);
  //  ukf.Setup(1.4, 1.6); 
  //  ukf.Setup(2.8, 3.2);
  //  ukf.Setup(0.56, 0.02);
  //  ukf.Setup(0.02*best_i, 0.02*best_j);
  //  ukf.Setup(0.3, 0.3);
  //  ukf.Setup(2.7+best_i*0.01, 3.1+best_j*0.01);

  //  ukf.Setup(best_i*0.2, best_j*0.2);
  for (size_t k = 0; k < number_of_measurements; ++k) {

    // Call the UKF-based fusion
    ukf.ProcessMeasurement(measurement_pack_list[k]);
    //    cout << k << ") " << ukf.x_ << endl;

    // output the estimation
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation

      // p1 - meas
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";

      // p2 - meas
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // p2_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\t";

    // output the NIS values
    
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      out_file_ << ukf.NIS_laser_ << "\n";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      out_file_ << ukf.NIS_radar_ << "\n";
    }

    // convert ukf x vector to cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(4);

    float x_estimate_ = ukf.x_(0);
    float y_estimate_ = ukf.x_(1);
    float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
    float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));

    //        cout << x_estimate_ << ", " << y_estimate_ << " estimated" << endl;
    //        cout << gt_pack_list[k].gt_values_(0) << ", "; 
    //        cout << gt_pack_list[k].gt_values_(1) << " actual" << endl; 
    
    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
    
    estimations.push_back(ukf_x_cartesian_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);

  }

  // compute the accuracy (RMSE)
    cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }
  cout << "Done!" << endl;
  return 0;
}
