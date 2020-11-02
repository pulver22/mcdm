#include "RadarModel.hpp"
#include "kalman-cpp/kalman.hpp"

#include <iostream>
#include <vector>
#include <boost/random.hpp>
#include <boost/nondet_random.hpp>
#include <chrono>
#include <Eigen/Dense>


const int NARGS = 5;
int totalReading = 0;
std::vector<std::pair<double,double>> getRobotDynamic(double resolution, double start_x, double start_y);
std::vector<std::pair<double,double>> getTagDynamic(double resolution,double absTag_X, double absTag_Y);
KalmanFilter createKalmanFilter(int n_states, int n_measurements, int numReadings);
void calculateAccuracy(std::vector<std::pair<double,double>> tags_coord, RadarModel* rm);

// quick build:
// Create build folder and go there
// cmake .. 
// make
//


/*
// Create animations ...

ffmpeg -r 10 -f image2 -s 1920x1080 -start_number 009 -i T0_S%03d_tempMap.png -vcodec libx264 -crf 25  -pix_fmt yuv420p test0.mp4

ffmpeg -r 10 -f image2 -s 1920x1080 -start_number 009 -i T1_S%03d_tempMap.png -vcodec libx264 -crf 25  -pix_fmt yuv420p test1.mp4

ffmpeg -r 10 -f image2 -s 1920x1080 -start_number 009 -i T2_S%03d_tempMap.png -vcodec libx264 -crf 25  -pix_fmt yuv420p test2.mp4

ffmpeg -r 10 -f image2 -s 1920x1080 -start_number 009 -i T3_S%03d_tempMap.png -vcodec libx264 -crf 25  -pix_fmt yuv420p test3.mp4


// gimp coords to RIC coords
./mcdm/Images/mfc_test.pgm
120 p x 200 p (gimp) == 200 p x 120 p (ric) 

*/

int main(int argc, char **argv)
{

  boost::random::random_device                  rand_dev;
  boost::random::mt19937                        generator(rand_dev());


  std::string mapFileURI = "/home/pulver/projects/mcdm/Images/mfc_test.pgm";
  double resolution = 1;
  double sigma_power = 4;
  double sigma_phase = 0.2;
  double txtPower = 0;
  double motionModelStdDev = 1.0;
  int i = 0;
  bool useKalman = false;
  cout <<"You provided: (" << argc-1 << ") arguments"<<  std::endl;

  if (argc > NARGS) {
            mapFileURI = argv[++i];
            resolution = atof(argv[++i]);
            sigma_power = atof(argv[++i]);
            sigma_phase = atof(argv[++i]);
            txtPower = atof(argv[++i]);
  } else {
      cout <<"DEFAULTS! I will execute call equivalent to:" << std::endl;
      cout << argv[0]      << " " << mapFileURI   << " " << resolution  << " " 
           << sigma_power  << " " << sigma_phase  << " " << txtPower    << " " << std::endl;
  }
  cout <<"PARAMETERS: " << std::endl;
  // Radio model  ........................................................
  cout <<"\tReference Map file: ["<< mapFileURI <<"]" << std::endl;
  cout <<"\tMap Resolution: ("<< resolution <<") m./cell" << std::endl;
  cout <<"\tPower Noise std: ("<< sigma_power <<")" << std::endl;
  cout <<"\tPhase Noise std: ("<< sigma_phase <<")" << std::endl;
  cout <<"\tTransmitted power: ("<< txtPower <<") dB." << std::endl;
    
  // loading tag poses ........................................................
  // Unit is meters. We multiply pixels by resolution to get them.
  // 0,0 is at upper left of the map, with X increasing down and Y increasing right.

  //green
  double absTag1_X = 37 * resolution;
  double absTag1_Y = 58 * resolution;

  //blue
  double absTag2_X = 44 * resolution;
  double absTag2_Y = 31 * resolution;
  
  // red
  double absTag3_X = 130 * resolution;
  double absTag3_Y = 45 * resolution;
  
  // yellow
  double absTag4_X = 150 * resolution;
  double absTag4_Y =  80 * resolution;

  std::vector<std::pair<double,double>> tags_coord;
  tags_coord.push_back(std::make_pair(absTag1_X, absTag1_Y));
  tags_coord.push_back(std::make_pair(absTag2_X, absTag2_Y));
  tags_coord.push_back(std::make_pair(absTag3_X, absTag3_Y));
  tags_coord.push_back(std::make_pair(absTag4_X, absTag4_Y));
  //  ........................................................
  double f_i =  MIN_FREQ_NA; // 902e6 Hertzs

  // create frequencies vector from possible ones:
  
  /*
  Model with ALL freqs takes 1 sec to start. Too long for short tests
  std::vector<double> freqs;
  do{
    freqs.push_back(f_i);
    f_i+=STEP_FREQ_NA;
  } while (f_i<=MAX_FREQ_NA);
  */

  // std::vector<double> freqs{ MIN_FREQ_NA,MIN_FREQ_NA+STEP_FREQ_NA,MIN_FREQ_NA+2.0*STEP_FREQ_NA }; 
  std::vector<double> freqs{ 800e6,920e6  }; 

  // later on, we will use this to get random freqs...
  boost::random::uniform_int_distribution<>  distr(0, freqs.size()-1);

  //  ........................................................


  cout <<"Building radar model." << endl;
  RadarModel rm(resolution, sigma_power, sigma_phase, txtPower, freqs, tags_coord, mapFileURI, motionModelStdDev);
  cout << "Radar model built." << endl;

  // prints reference map with tags
  rm.PrintRefMapWithTags("/tmp/test/scenario.png");      
  //  - Map goes from 0,0 to (N,M)*resolution in meters
  //  - Rcoods: up-left is 0,0. X increases down, and Y increases Right: swap x and y axes from images as they show on gimp 

  //rm.PrintRecPower("/tmp/test/rec_power_f_800.png", 800e6);
  //rm.PrintRecPower("/tmp/test/rec_power_f_920.png", 920e6);

  //rm.PrintPowProb("/tmp/test/prob_rec_power_95_f_800.png", -95, 800e6);
  //rm.PrintPowProb("/tmp/test/prob_rec_power_95_f_920.png", -95, 920e6);

  // Draw different probabilities distribution
  // std::string fileURI = "/tmp/test/prob_rec_power_";
  // char buffer [50];
  // int n;

  // for (double rxPi=txtPower;rxPi>SENSITIVITY;rxPi = rxPi-5.0){
  //     n=sprintf (buffer, "%d", (int) abs(rxPi));
  //     fileURI += std::string(buffer)+ "_f_920.png";
  //     rm.PrintPowProb(fileURI, rxPi, 920e6);
  //     fileURI = "/tmp/test/prob_rec_power_";
  // }
  
  //rm.PrintPhase("/tmp/test/phase_f_800.png", 800e6);
  //rm.PrintPhase("/tmp/test/phase_f_920.png", 920e6);

  //rm.PrintPhaseProb("/tmp/test/prob_phase_45_f_800.png", 45.0*M_PI/180.0, 800e6);
  //rm.PrintPhaseProb("/tmp/test/prob_phase_45_f_920.png", 45.0*M_PI/180.0, 920e6);

  //rm.PrintBothProb("/tmp/test/prob_95db_45deg_f_800.png", -95, 45.0*M_PI/180.0, 800e6);
  //rm.PrintBothProb("/tmp/test/prob_95db_45deg_f_920.png", -95, 45.0*M_PI/180.0, 920e6);
  
  // simple scenario
  // int NumReadings;
  // Unit is meters. We multiply pixels by resolution to get them.
  double start_x = 125 * resolution;
  double start_y = 65 * resolution;
  // double end_x = 127 * resolution;
  // double end_y = 46 * resolution;
  
  double robot_y;
  double robot_x;
  double robot_head0=0;// atan2( end_y - start_y, end_x - start_x);
  double robot_head;
  double rxPower ;
  double tag_x, delta_x;
  double tag_y, delta_y;
  double phase;

  for (int t = 0; t < tags_coord.size(); t++){
    rm.saveProbMapDebug("/tmp/test/",t,0,start_x,start_y, robot_head0);
  }

  // ROBOT Dynamic  ////////////////////////////////////////
  // build sampling poses
  std::vector<std::pair<double,double>> robot_poses = getRobotDynamic(resolution, start_x, start_y);


  // TAG Dynamic ////////////////////////////////////////
  std::vector<std::pair<double,double>> tag1_poses = getTagDynamic(resolution, absTag1_X, absTag1_Y);
  
  ///////////////////////////////////////////////////////
  KalmanFilter kf;
  std::pair<int, std::pair<int, int>> tag_measurement; // <powerRead, tag_position(x,y)>>
  int n_states = 4; // Number of states
  int n_measurements = 2; // Number of measurements
  Eigen::VectorXd y(n_measurements);
  
  if (useKalman){
    // KALMAN FILTER SETUP ///////////////////////////////
    kf = createKalmanFilter(n_states, n_measurements, totalReading);
    // Feed measurements into filter, output estimated states
    tag_measurement = rm.findTagFromBeliefMap(0);
    // Best guess of initial states
    Eigen::VectorXd x0(n_states);
    // x0 << measurements[0], 0, -9.81;
    x0[0] = tag_measurement.second.first;
    x0[1] = tag_measurement.second.second;
    x0[2] = 0;
    x0[3] = 0;
    // Initialize the Kalman filter
    double t = 0;
    kf.init(t, x0);
  ///////////////////////////////////////////////////////
  }
  

std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
int num_ops = 0;
string tagLayerName;
  // let's do the path ...
  
  cout << "\nInitial pose tag1: " << tags_coord[0].first << "," << tags_coord[0].second << endl;
  for (int i = 0; i < robot_poses.size(); i++){
        robot_x = robot_poses[i].first;
        robot_y =  robot_poses[i].second;

      // Find the tag and use it as measurement for the KF
      if (useKalman){
        tag_measurement = rm.findTagFromBeliefMap(0);
        y[0] = (tag_measurement.second).first;
        y[1] = (tag_measurement.second).second;
        kf.update(y);
      
      
        // Update Tag1 position
        // tags_coord[0] = tag1_poses.at(i);
        // cout << "   [" << i <<"]tag1: " << tags_coord[0].first << "," << tags_coord[0].second << endl;
        //         cout << "     [" << i <<"-KF]tag1: " << kf.state().transpose()[0] << "," << kf.state().transpose()[1] << endl;
        //         cout << "   [measurement]: " << tag_measurement.second.first << ", " << tag_measurement.second.second << endl;
        // rm.updateTagsPosition(tags_coord);
      }

      // 8 orientations at each pose
      for (int h=0;h<9;h++){
        robot_head = robot_head0 + (2.0*M_PI* h/9);
        // std::cout<<"Robot at (" << robot_x << ", " << robot_y <<") m., (" << (robot_head*180.0/M_PI) << ") deg." << std::endl;
        
        //for each tag:
        for (int t = 0; t < tags_coord.size(); t++){
            // get relative robot-tag pose
            // translate
            delta_x = (tags_coord[t].first - robot_x ); 
            delta_y = (tags_coord[t].second - robot_y);
            // rotate
            tag_x =  delta_x * cos(robot_head) + delta_y * sin(robot_head);
            tag_y = -delta_x * sin(robot_head) + delta_y * cos(robot_head);

            // std::cout<<"\t- Tag [" << t << "] at position (" << tags_coord[t].first << ", " << tags_coord[t].second << ") m. " <<std::endl;
            // std::cout<<"\t\t  rel position (" << tag_x << ", " << tag_y << ") m. " <<std::endl;

            // get expected tag power with friis
            f_i = freqs[distr(generator)]; 
            
            rxPower = rm.received_power_friis_with_obstacles(robot_x, robot_y, robot_head, tags_coord[t].first , tags_coord[t].second, 0, f_i);
            num_ops++;

            // get expected phase from tag
            phase = rm.phaseDifference( tag_x,  tag_y,  f_i);
            //std::cout<<"\tReading at freq (" << f_i/1e6<< " MHz): (" << (rxPower+30) << ") dBm. ( " << phase << ") rads. " << std::endl << std::endl;
            tagLayerName = rm.getTagLayerName(t);
            rm.addMeasurement(robot_x, robot_y, robot_head*180.0/M_PI,  rxPower,  phase,  f_i,  tagLayerName);
            
            //print maps
            //cout  << "Saving tag distribution maps... "<< endl;
            int lineal_index = (8*(i+1))+(h+1);
            rm.saveProbMapDebug("/tmp/test/",t,lineal_index,robot_x,robot_y, robot_head);
        }
        //std::cout<<"Finished reading. " << std::endl << std::endl;
    }
  }

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_span =  std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
  std::cout << num_ops << " measurements took " 
                      << time_span.count() << " [secs]" << std::endl 
                      << std::endl;
                      
  //print maps
  rm.saveProbMaps("/tmp/test/");
  // for each tag:
  for (int t = 0; t < tags_coord.size(); t++){
    // cout << "---[" << t <<"]----------------" << endl;
   rm.saveProbMapDebug("/tmp/",t,0,0,0,0);
  }

  

  // lets play with the weights
  double w_i, w_max;
  double prec = 3;
  std::cout << std::fixed;
  std::cout << std::setprecision(2);

  std::cout << "Tag probability of being in " << (prec) << " square meters around different poses: " << std::endl;
  std::cout << "(Which happen to be tag reference poses...) " << std::endl;

  std::string sep = "\t\t";
    std::cout << "Tag \\ Pos" << "\t";
    for (int t = 0; t < tags_coord.size(); t++){
        tag_x = tags_coord[t].first;
        tag_y = tags_coord[t].second;
        std::cout << tag_x << ", " << tag_y << "\t";
  }
  std::cout << std::endl;

  for (int i = 0; i < tags_coord.size(); i++){
    cout  << " " << (i+1) << sep;
    w_max = rm.getTotalWeight(i);
    for (int t = 0; t < tags_coord.size(); t++){
        tag_x = tags_coord[t].first;
        tag_y = tags_coord[t].second;

        //w_i = rm.getTotalWeight(tag_x, tag_y, 0, i); // this uses the whole "active area" (i.e. 24x12 meters ) ... I don't think is really useful ...
        w_i = rm.getTotalWeight(tag_x, tag_y,0, sqrt(prec),sqrt(prec), i);
        cout  << (100.0 * w_i/w_max) << sep;
    }
    std::cout << std::endl;
  }

  // Calculate the localization accuracy of the tags.
  calculateAccuracy(tags_coord, &rm);

  return 0;
}


std::vector<std::pair<double,double>> getRobotDynamic(double resolution,
                                        double start_x, double start_y){
  std::vector<std::pair<double,double>> robot_poses;
  int NumReadings = 5;
  double end_x = 127 * resolution;
  double end_y = 46 * resolution;
  
  double robot_y;
  double robot_x;
  double robot_head0=0;// atan2( end_y - start_y, end_x - start_x);
  double robot_head;
  for (int i=0;i<NumReadings;i++){
      // current robot position, map coordinates.   
      robot_x = start_x + ( (end_x - start_x) * ( i ) / (NumReadings - 1.0) );
      robot_y = start_y + ( (end_y - start_y) * ( i ) / (NumReadings - 1.0) );
      robot_poses.push_back(std::make_pair(robot_x, robot_y));
      totalReading++;
  }

  // other 10.
  NumReadings = 10;
  start_x = end_x;
  start_y = end_y;
  end_x = 45 * resolution;
  end_y = 32 * resolution;
  for (int i=0;i<NumReadings;i++){
      // current robot position, map coordinates.   
      robot_x = start_x + ( (end_x - start_x) * ( i ) / (NumReadings - 1.0) );
      robot_y = start_y + ( (end_y - start_y) * ( i ) / (NumReadings - 1.0) );
      robot_poses.push_back(std::make_pair(robot_x, robot_y));
      totalReading++;
  }

  // other 5.
  NumReadings = 5;
  start_x = end_x;
  start_y = end_y;
  end_x = 40 * resolution;
  end_y = 55 * resolution;
  for (int i=0;i<NumReadings;i++){
      // current robot position, map coordinates.   
      robot_x = start_x + ( (end_x - start_x) * ( i ) / (NumReadings - 1.0) );
      robot_y = start_y + ( (end_y - start_y) * ( i ) / (NumReadings - 1.0) );
      robot_poses.push_back(std::make_pair(robot_x, robot_y));
      totalReading++;
  }

  // other 10.
  NumReadings = 10;
  start_x = end_x;
  start_y = end_y;
  end_x = 125 * resolution;
  end_y = 75 * resolution;
  for (int i=0;i<NumReadings;i++){
      // current robot position, map coordinates.   
      robot_x = start_x + ( (end_x - start_x) * ( i ) / (NumReadings - 1.0) );
      robot_y = start_y + ( (end_y - start_y) * ( i ) / (NumReadings - 1.0) );
      robot_poses.push_back(std::make_pair(robot_x, robot_y));
      totalReading++;
  }

  // other 10.
  NumReadings = 10;
  start_x = end_x;
  start_y = end_y;
  end_x = 44 * resolution;
  end_y = 55 * resolution;
  for (int i=0;i<NumReadings;i++){
      // current robot position, map coordinates.   
      robot_x = start_x + ( (end_x - start_x) * ( i ) / (NumReadings - 1.0) );
      robot_y = start_y + ( (end_y - start_y) * ( i ) / (NumReadings - 1.0) );
      robot_poses.push_back(std::make_pair(robot_x, robot_y));
      totalReading++;
  }

  // other 10.
  NumReadings = 10;
  start_x = end_x;
  start_y = end_y;
  end_x = 45 * resolution;
  end_y = 32 * resolution;
  for (int i=0;i<NumReadings;i++){
      // current robot position, map coordinates.   
      robot_x = start_x + ( (end_x - start_x) * ( i ) / (NumReadings - 1.0) );
      robot_y = start_y + ( (end_y - start_y) * ( i ) / (NumReadings - 1.0) );
      robot_poses.push_back(std::make_pair(robot_x, robot_y));
      totalReading++;
  }


  // other 10.
  NumReadings = 10;
  start_x = end_x;
  start_y = end_y;
  end_x = 127 * resolution;
  end_y = 46 * resolution;
  for (int i=0;i<NumReadings;i++){
      // current robot position, map coordinates.   
      robot_x = start_x + ( (end_x - start_x) * ( i ) / (NumReadings - 1.0) );
      robot_y = start_y + ( (end_y - start_y) * ( i ) / (NumReadings - 1.0) );
      robot_poses.push_back(std::make_pair(robot_x, robot_y));
      totalReading++;
  }
  return robot_poses;
}

std::vector<std::pair<double,double>> getTagDynamic(double resolution, double absTag_X, double absTag_Y){
  std::vector<std::pair<double,double>> tag_poses;
  // 5 samples from start point and ing a straight line.
  int NumReadings = totalReading;
  double start_x = absTag_X;
  double start_y = absTag_Y;
  double end_x = 46 * resolution;
  double end_y = 32 * resolution;
  double tag_x, tag_y;
  for (int i=0;i<NumReadings;i++){
    // current robot position, map coordinates.   
    tag_x = start_x + ( (end_x - start_x) * ( i ) / (NumReadings - 1.0) );
    tag_y = start_y + ( (end_y - start_y) * ( i ) / (NumReadings - 1.0) );
    tag_poses.push_back(std::make_pair(tag_x, tag_y));
  }
  return tag_poses;
}

KalmanFilter createKalmanFilter(int n_states, int n_measurements, int numReadings){
  // KALMAN FILTER SETUP ///////////////////////////////
  int n = n_states; // Number of states
  int m = n_measurements; // Number of measurements

  double dt = 1.0/numReadings; // Time step
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance (external uncertainty)
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;
  C << 1, 0, 0, 0, 0, 1, 0, 0;

  // Reasonable covariance matrices
  Q << .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, 0.5;
  R <<  0.1, 0.1, 0.1, 0.1;
  P << .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 10000, 100, .1, .1, 100, 10000;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt, A, C, Q, R, P);
  return kf;
}

void calculateAccuracy(std::vector<std::pair<double,double>> tags_coord, RadarModel* rm){
  std::pair<double, double> belief_tag;
  std::pair<int, std::pair<int, int>> belief_value_tag;
  double belief_distance_to_tag = 0;
  double belief_accuracy = 0;
  for (int i=0; i < tags_coord.size(); i++){
    belief_value_tag = rm->findTagFromBeliefMap(i);
    belief_tag = belief_value_tag.second;
    // cout << "Before: " << belief_tag.first;
    // belief_tag.first = (200.0 - abs(belief_tag.first))*resolution;
    // cout << "After: " << belief_tag.first;
    // belief_tag.second = (120.0 - abs(belief_tag.second))*resolution;
    belief_distance_to_tag = sqrt(pow((tags_coord)[i].first - belief_tag.first, 2) + 
                                  pow((tags_coord)[i].second - belief_tag.second, 2));
    std::cout << "------" << "[" << i << "]------" << endl;
    cout << "T: " << tags_coord[i].first << "," << tags_coord[i].second << endl;
    cout << "B: " << belief_tag.first << "," << belief_tag.second << endl;
    // std::cout << "[GT]     Tag: " << to_string((*tags_coord)[i].first) << ", " << to_string((*tags_coord)[i].second) << endl;
    // std::cout << "[Belief] Tag: " << belief_tag.first << "," << belief_tag.second << endl;
    std::cout << "Belief_Distance to tag: " << to_string(belief_distance_to_tag) << " cells" << endl;
    if (belief_distance_to_tag <= 10.0)
    {
      belief_accuracy = belief_accuracy + 1;
      // std::cout << "Tag: " << i << " found" << std::endl;
    } 
  }
  belief_accuracy = belief_accuracy / tags_coord.size();
  std::cout << "Belief_Accuracy: " << to_string(belief_accuracy) << endl;

}

/*

  //rm.getImage("P_910.00", "/tmp/Average_power_map_f_910.png");            
  //rm.getImage("D_910.00", "/tmp/Average_phase_map_f_910.png");  
  //rm.PrintPowProb("/tmp/prob_rec_power_f_910.png", test_rxp, f_i);          
  //rm.PrintPhaseProb("/tmp/prob_rec_phase_f_910.png", test_phase, f_i);          
  
  
  // export tag locations into image, overimpressing background map.
  // at least points should be in in white areas1=1 




To read an arbitrary long tag loc list

  double absTag_pose_x;
  std::string absTag_string_x;
  double absTag_pose_y;
  std::string absTag_string_y;

  // Create an output string stream
  std::ostringstream streamObj3;

  // Set Fixed -Point Notation
  streamObj3 << std::fixed;

  // Set precision to 2 digits
  streamObj3 << std::setprecision(2);

  std::vector<std::pair<double,double>> tags_coord;
  for (int i=1;i<=numTags){
    // flush streamObj3
    streamObj3.clear();
    //streamObj3.str("");
    streamObj3 << numTags;

    absTag_string_x = streamObj3.str();
    absTag_string_y = streamObj3.str();
    absTag_pose_x = config[absTag_string_x].as<double>();
    absTag_pose_y = config[absTag_string_y].as<double>();
    tags_coord.push_back(std::make_pair(absTag_pose_x, absTag_pose_y));
  }

*/
