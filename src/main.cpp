
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include <string>
#include <unistd.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

struct globalArgs_t {
  bool debug;                 /* print out arguments but don't do anything else */
  bool search;                /* search for optimal acceleration values */
  bool radar;                 /* use radar; cleared by -l option that chooses laser*/
  bool laser;                 /* use laser; cleared by -m option that chooses radar*/
  char **inputFiles;          /* input files */
  int numInputFiles;          /* # of input files */
  bool valid;                 /* valid args */
  float a_straight;           /* straight acceleration */
  float a_turn;               /* turn acceleration */
} globalArgs;
 
static const char *optString = "dsrlha:A:";

bool isFloat(string s) {
  istringstream iss(s);
  float dummy;
  iss >> noskipws >> dummy;
  return iss && iss.eof();     // Result converted to bool
}

void display_usage(char* argv[]) {
  cout << "Usage: " << argv[0] << " options* path/to/input.txt output.txt" << endl;
  cout << "  -d debug command line arguments" << endl;
  cout << "  -h help" << endl;
  cout << "  -a <float> standard deviation in acceleration when driving straight" << endl;
  cout << "  -A <float> standard deviation in acceleration when turning" << endl;
  cout << "  -l only use laser, e.g., turn off radar" << endl;
  cout << "  -r only use radar, e.g., turn off laser" << endl;
  cout << "  -s search for optimal acceleration values " << endl;
  exit(EXIT_FAILURE);
}

void show_args(string in_file_name_, string out_file_name_) {
  cout << "Input: " << in_file_name_ << endl;
  cout << "Output: " << out_file_name_ << endl;
  cout << "Search: " << globalArgs.search << endl;
  cout << "Radar: " << globalArgs.radar << endl;
  cout << "Laser: " << globalArgs.laser << endl;
  cout << "Straight accel: " << globalArgs.a_straight << endl;
  cout << "Turn accel: " << globalArgs.a_turn << endl;
  exit(0);
}

void init_args() {
  globalArgs.debug = false;
  globalArgs.search = false;
  globalArgs.radar = true;
  globalArgs.laser = true;
  globalArgs.inputFiles = NULL;
  globalArgs.numInputFiles = 0;
  globalArgs.valid = true;
  globalArgs.a_straight = 0.6;  // 2.7
  globalArgs.a_turn = 1.8;      // 3.2
}

void check_arguments(int argc, char* argv[]) {

  init_args();
  int opt;
  float accel;

  while ((opt = getopt(argc, argv, optString)) != -1) {
    switch (opt) {
    case 'a':
      if (isFloat(optarg)) {
	sscanf(optarg, "%f", &accel);
	globalArgs.a_straight = accel;
      }
      else {
	cout << "Linear acceleration -a must be a floating point number." << endl;
	globalArgs.valid = false;
      }
      break;
    case 'A':
      if (isFloat(optarg)) {
	sscanf(optarg, "%f", &accel);
	globalArgs.a_turn = accel;
      }
      else {
	cout << "Turn acceleration -A must be a floating point number." << endl;
	globalArgs.valid = false;
      }
      break;
    case 'd':
      globalArgs.debug = true;
      break;
    case 's':
      globalArgs.search = true;
      break;
    case 'l':
      globalArgs.radar = false;
      break;
    case 'r':
      globalArgs.laser = false;
      break;
    case 'h':
    default:
      cout << "default handling" << endl;
      display_usage(argv);
    }
  }

  globalArgs.inputFiles = argv + optind;
  globalArgs.numInputFiles = argc - optind;

  // make sure the user has provided input and output files
  if (globalArgs.numInputFiles < 1) {
    globalArgs.valid = false;
  } else if (globalArgs.numInputFiles == 1) {
    cerr << "Please include an output file." << endl;
    globalArgs.valid = false;
  } else if (globalArgs.numInputFiles > 2) {
    cerr << "Too many arguments." << endl;
    globalArgs.valid = false;
  }

  if (!globalArgs.valid) {
    display_usage(argv);
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

bool skip_measurement(MeasurementPackage m) {
  if (m.sensor_type_ == MeasurementPackage::LASER) {
    return fabs(m.raw_measurements_[0]) < 0.0001;
  }
  if (m.sensor_type_ == MeasurementPackage::RADAR) {
    return fabs(m.raw_measurements_[2]) < 0.0001;
  }
  return false;
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);
  
  string in_file_name_ = globalArgs.inputFiles[0];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = globalArgs.inputFiles[1];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  if (globalArgs.debug) {
    show_args(in_file_name_, out_file_name_);
  }

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

  if (globalArgs.search) {
    for (int i=0; i < 40; i++) {
      cout << "**Loop " << i << endl;
      for (int j=0; j < 40; j++) {

	ukf.Setup(i*0.1, j*0.1);
	  //	ukf.Setup(2.7+i*0.01, 3.1+j*0.01);
	
	for (size_t k = 0; k < number_of_measurements; ++k) {
	  if (skip_measurement(measurement_pack_list[k])) continue;
	  
	  bool isLaser = (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER);
	  bool isRadar = (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR);
	  bool skip = (isLaser && !globalArgs.laser) || (isRadar && !globalArgs.radar);

	  // Call the UKF-based fusion
	  ukf.ProcessMeasurement(measurement_pack_list[k], skip);
	  
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
	  cout << "Using accel straight " << (0.1*i) << " turn " << (0.1*j) << endl;
	}
	
	estimations.clear();
	ground_truth.clear();
      }
    }
  }
  //  ukf.Setup(0.6, 0.1) // for second data set
  //  ukf.Setup(2.8, 3.2) // for first data set

  if (globalArgs.search) {
    ukf.Setup(0.1*best_i, 0.1*best_j);
  }
  else {
    ukf.Setup(globalArgs.a_straight, globalArgs.a_turn);
  }

  int radar_count = 0;
  int ok_nis_count = 0;
  
  for (size_t k = 0; k < number_of_measurements; ++k) {
    if (skip_measurement(measurement_pack_list[k])) continue;

    bool isLaser = (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER);
    bool isRadar = (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR);
    bool skip = (isLaser && !globalArgs.laser) || (isRadar && !globalArgs.radar);

    // Call the UKF-based fusion
    ukf.ProcessMeasurement(measurement_pack_list[k], skip);

    // output the estimation
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // output the measurements
    if (isLaser) {
      // output the estimation

      // p1 - meas
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";

      // p2 - meas
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (isRadar) {
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
    
    if (isLaser) {
      out_file_ << ukf.NIS_laser_ << "\n";
    } else if (isRadar) {
      radar_count++;
      if ((ukf.NIS_radar_ >= 0.35) && (ukf.NIS_radar_ <= 7.81)) {
	ok_nis_count++;
      }
      else {
	//	cout << "Bad NIS " << ukf.NIS_radar_ << endl;
      }
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

  // show NIS
  if (radar_count < 1) {
    cout << "No radar readings, no NIS to report." << endl;
  }
  else {
    cout << "Valid NIS: " << (float(ok_nis_count)/float(radar_count))*100 << "% " << endl;
  }

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
