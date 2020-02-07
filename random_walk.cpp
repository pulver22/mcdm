#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>
#include "map.h"
#include "newray.h"
#include "mcdmfunction.h"
#include "Criteria/traveldistancecriterion.h"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>
#include <time.h>
#include <ctime>
#include "utils.h"
// #include "RFIDGridmap.h"
#include "yaml-cpp/yaml.h"
#include "RadarModel.hpp"
#include <vector>




using namespace std;
using namespace dummy;
using namespace YAML;

// Example: ./mcdm_online_exploration ./../Images/cor_map_05_00_new1.pgm 1 99 99 180 5 180 1 0 1 54 143 865e6 0
int main ( int argc, char **argv )
{
  auto startMCDM = chrono::high_resolution_clock::now();
  ifstream infile;
  infile.open ( argv[1] );  // the path to the map
  double resolution = atof ( argv[2] );  // the resolution of the map
  double imgresolution = atof ( argv[10] );  // the resolution to use for the planningGrid and RFIDGrid
  dummy::Map map = dummy::Map ( infile,resolution, imgresolution );
  // cout << "Map dimension: " << map.getNumGridCols() << " : "<<  map.getNumGridRows() << endl;
  int gridToPathGridScale = map.getGridToPathGridScale();
  // i switched x and y because the map's orientation inside and outside programs are different
  long initX = static_cast<long>( atoi ( argv[4] ) *imgresolution );  // initial X-position of the robot in map frame
  long initY = static_cast<long>( atoi ( argv[3] ) *imgresolution );  // initial Y-position of the robot in map frame
  // std::cout << "initX: " << initX << " initY: " << initY << std::endl;
  int initOrientation = atoi ( argv[5] );  // initial orientation of the robot in map frame
  double initFov = atoi ( argv[7] );  // initial FOV of the robot sensor
  initFov = initFov * PI /180;
  int initRange = atoi ( argv[6] );
  double precision = atof ( argv[8] );
  double threshold = atof ( argv[9] );
  /// RFID
  YAML::Node config = YAML::LoadFile(argv[11]);
  
  double absTag1_X = config["tag1X"].as<double>();
  double absTag1_Y = config["tag1Y"].as<double>();

  double absTag2_X = config["tag2X"].as<double>();
  double absTag2_Y = config["tag2Y"].as<double>();

  double absTag3_X = config["tag3X"].as<double>();
  double absTag3_Y = config["tag3Y"].as<double>();

  double absTag4_X = config["tag4X"].as<double>();
  double absTag4_Y = config["tag4Y"].as<double>();

  double absTag5_X = config["tag5X"].as<double>();
  double absTag5_Y = config["tag5Y"].as<double>();

  double absTag6_X = config["tag6X"].as<double>();
  double absTag6_Y = config["tag6Y"].as<double>();

  double absTag7_X = config["tag7X"].as<double>();
  double absTag7_Y = config["tag7Y"].as<double>();

  double absTag8_X = config["tag8X"].as<double>();
  double absTag8_Y = config["tag8Y"].as<double>();

  double absTag9_X = config["tag9X"].as<double>();
  double absTag9_Y = config["tag9Y"].as<double>();

  double absTag10_X = config["tag10X"].as<double>();
  double absTag10_Y = config["tag10Y"].as<double>();

  std::vector<std::pair<double,double>> tags_coord;
  tags_coord.push_back(std::make_pair(absTag1_X, absTag1_Y));
  tags_coord.push_back(std::make_pair(absTag2_X, absTag2_Y));
  tags_coord.push_back(std::make_pair(absTag3_X, absTag3_Y));
  tags_coord.push_back(std::make_pair(absTag4_X, absTag4_Y));
  tags_coord.push_back(std::make_pair(absTag5_X, absTag5_Y));
  tags_coord.push_back(std::make_pair(absTag6_X, absTag6_Y));
  tags_coord.push_back(std::make_pair(absTag7_X, absTag7_Y));
  tags_coord.push_back(std::make_pair(absTag8_X, absTag8_Y));
  tags_coord.push_back(std::make_pair(absTag9_X, absTag9_Y));
  tags_coord.push_back(std::make_pair(absTag10_X, absTag10_Y));

  RFIDGridmap myGrid1(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid2(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid3(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid4(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid5(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid6(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid7(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid8(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid9(argv[1], resolution, imgresolution, false);
  RFIDGridmap myGrid10(argv[1], resolution, imgresolution, false);
  std::vector<RFIDGridmap> RFID_maps_list;
  RFID_maps_list.push_back(myGrid1);
  RFID_maps_list.push_back(myGrid2);
  RFID_maps_list.push_back(myGrid3);
  RFID_maps_list.push_back(myGrid4);
  RFID_maps_list.push_back(myGrid5);
  RFID_maps_list.push_back(myGrid6);
  RFID_maps_list.push_back(myGrid7);
  RFID_maps_list.push_back(myGrid8);
  RFID_maps_list.push_back(myGrid9);
  RFID_maps_list.push_back(myGrid10);

  double freq = std::stod(argv[12]); // Hertzs
  double txtPower = std::stod(argv[13]); // dBs
  std::pair<int, int> relTagCoord;

  // MCDM Matrix weights
  double w_info_gain = atof(argv[14]);
  double w_travel_distance = atof(argv[15]);
  double w_sensing_time = atof(argv[16]);
  double w_rfid_gain = atof(argv[17]);
  double w_battery_status = atof(argv[18]);
  std::string out_log (argv[19]);
  std::string coverage_log (argv[20]);
  std::string detection_log (argv[21]);
  std::string accuracy_log (argv[23]);
  bool use_mcdm = bool(atoi(argv[24]));
  //x,y,orientation,range,FOV
  double norm_w_info_gain, norm_w_travel_distance, norm_w_sensing_time, norm_w_rfid_gain, norm_w_battery_status;
  double sum_w = w_info_gain + w_travel_distance + w_sensing_time + w_rfid_gain + w_battery_status;
  norm_w_info_gain = w_info_gain / sum_w;
  norm_w_travel_distance = w_travel_distance / sum_w;
  norm_w_sensing_time = w_sensing_time / sum_w;
  norm_w_rfid_gain = w_rfid_gain / sum_w;
  norm_w_battery_status = w_battery_status / sum_w;
  // cout << "[ " << norm_w_info_gain << ", " << norm_w_travel_distance 
  //       << ", " << norm_w_sensing_time << ", " << norm_w_rfid_gain << " ]" << endl;
  Utilities utils(norm_w_info_gain, norm_w_travel_distance, norm_w_sensing_time, norm_w_rfid_gain, norm_w_battery_status);

  Pose initialPose = Pose ( initX,initY,initOrientation,initRange,initFov );
  Pose invertedInitial = utils.createFromInitialPose ( initX,initY,initOrientation,180,initRange,initFov );
  Pose eastInitial = utils.createFromInitialPose ( initX,initY,initOrientation,90,initRange,initFov );
  Pose westInitial = utils.createFromInitialPose ( initX,initY,initOrientation,270,initRange,initFov );
  Pose target = initialPose;
  Pose previous = initialPose;
  long numConfiguration = 1;
  vector<pair<string,list<Pose>>> graph2;
  NewRay ray;
  ray.setGridToPathGridScale ( gridToPathGridScale );
  // MCDMFunction function(w_info_gain, w_travel_distance, w_sensing_time);
  MCDMFunction function(w_info_gain, w_travel_distance, w_sensing_time, w_rfid_gain);
  // cout << "MCDM matrix created! " << endl;
//  MCDMFunction function;
  long sensedCells = 0;
  long newSensedCells = 0;
  long totalFreeCells = map.getTotalFreeCells();
  int count = 0;
  double travelledDistance = 0;
  int numOfTurning = 0;
  unordered_map<string,int> visitedCell;
  vector<string>history;
  history.push_back ( function.getEncodedKey ( target,1 ) );
  EvaluationRecords record;
  //amount of time the robot should do nothing for scanning the environment ( final value expressed in second)
  unsigned int microseconds = 5 * 1000 * 1000 ;
  list<Pose> unexploredFrontiers;
  list<Pose> tabuList;
  tabuList.push_back(target);
  list<Pose> nearCandidates;
  bool btMode = false;
  double totalAngle = 0;
  Astar astar;
  double totalScanTime = 0;
  bool act = true;
  int encodedKeyValue = 0;
  string content;
  std::pair<long, long> nextRandomPosition;
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<> distr(0, 359); // define the range
   // Radar model: 
  double nx = 240*resolution; // radar model active area x-range m.
  double ny = 120*resolution;  // radar model active area y-range m.  
  double rs = resolution; // radar model grid resolution m./cell :: SAME AS INPUT IMAGE!!!
  double sigma_power = 1; //dB
  double sigma_phase = 1; //rads
  txtPower = txtPower; // NOTE: Added for debug
  std::vector<double> freqs{ freq }; // only 1 freq... noice!
  // std::vector<double> freqs{ MIN_FREQ_NA,MIN_FREQ_NA+STEP_FREQ_NA,MIN_FREQ_NA+2.0*STEP_FREQ_NA }; 

  cout <<"Building radar model." << endl;
  RadarModel rm(nx, ny, rs, sigma_power, sigma_phase, txtPower, freqs, tags_coord, argv[1] );
  cout << "Radar model built." << endl;
  rm.PrintRefMapWithTags("/tmp/scenario.png"); 

  long x, y = 0;
  int orientation, range;
  double FOV, major_axis, minor_axis, scanAngle, rxPower, phase;
  string encoding, actualPose;

  double accumulated_received_power = 0.0;
  double batteryTime = MAX_BATTERY;
  double distance = 0.0;
  double tmp_numOfTurning = 0.0;
  double translTime = 0.0;
  double rotTime = 0.0;

  RFID_tools rfid_tools;
  rfid_tools.rm = rm;
  rfid_tools.tags_coord = tags_coord;
  rfid_tools.freq = freq;
  rfid_tools.txtPower = txtPower; 

  do
  {
      content = to_string(w_info_gain) 
                + "," + to_string(w_travel_distance)
                + "," + to_string(w_sensing_time) 
                + "," + to_string(w_rfid_gain)
                + "," + to_string(w_battery_status)
                + "," + to_string(norm_w_info_gain)
                + "," + to_string(norm_w_travel_distance)
                + "," + to_string(norm_w_sensing_time)
                + "," + to_string(norm_w_rfid_gain)
                + "," + to_string(norm_w_battery_status)
                + "," + to_string(numConfiguration) 
                + "," + to_string(100 * float(newSensedCells)/float(totalFreeCells)) 
                + "," + to_string(travelledDistance) + "\n" ;
      utils.filePutContents(coverage_log, content, true );
      x = target.getX();
      y = target.getY();
      orientation = target.getOrientation();
      range = target.getRange();
      FOV = target.getFOV();
      actualPose = function.getEncodedKey ( target,0 );
      map.setCurrentPose ( target );
      // Update the overall covered distance
      string path = astar.pathFind ( target.getX(), target.getY(), previous.getX(), previous.getY(), &map );
      distance = astar.lengthPath(path);
      travelledDistance = travelledDistance + distance;
      tmp_numOfTurning = astar.getNumberOfTurning ( path );
      translTime = distance / TRANSL_SPEED;
      rotTime = tmp_numOfTurning / ROT_SPEED;
      batteryTime = batteryTime - (translTime + rotTime);
      // Update the overall number of turnings
      numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );
      visitedCell.emplace ( encoding,0 );
      // Get the sensing time required for scanning
      target.setScanAngles ( ray.getSensingTime ( &map,x,y,orientation,FOV,range ) );
      // Perform a scanning operation
      double X_max = range;
      double X_min = atoi ( argv[22] );
      double focal_length = (X_max - X_min) / 2.0; // (X_max - X_min)/2
      major_axis = focal_length + X_min;  // (focal_length + X_min)
      minor_axis = sqrt(pow(major_axis, 2) - pow(focal_length, 2));
      newSensedCells = sensedCells + ray.performSensingOperationEllipse ( &map,x,y,orientation,             target.getScanAngles().first, target.getScanAngles().second, major_axis, minor_axis);
      // Calculate the scanning angle
      scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      // Update the overall scanning time
      totalScanTime += utils.calculateScanTime ( scanAngle*180/PI );
      // Update bot the PP and the RFID maps
      utils.updateMaps(&tags_coord, &map, &target, &txtPower, &SENSITIVITY, &freq, &RFID_maps_list, &x, &y, range, &rfid_tools);
      // Calculate the accumulated received power
      for (int tag_id = 0; tag_id < tags_coord.size(); tag_id++){
        accumulated_received_power += rfid_tools.rm.received_power_friis(tags_coord[tag_id].first, tags_coord[tag_id].second, freq, txtPower);
      }
      // Find new random destination
      nextRandomPosition = map.getRandomFreeCell();
      orientation = distr(eng);  // get a random orientation between 0 and 359
      orientation = orientation * M_PI / 180.0;
      target.updateFromData(nextRandomPosition, orientation, range, FOV);
      sensedCells = newSensedCells;
      numConfiguration++;
      // Add target to history and tabulist
      history.push_back(function.getEncodedKey(target, 1));
      tabuList.push_back(target);

  }
  // Perform exploration until a certain coverage is achieved
  while ( sensedCells < precision * totalFreeCells and batteryTime > 0.0 );
  // Plotting utilities
  // map.drawVisitedCells ();
  // map.printVisitedCells ( history );
  // map.drawRFIDScan();
  // map.drawRFIDGridScan(myGrid);
  // myGrid.saveAs(("/home/pulver/Desktop/MCDM/rfid_result_gridmap.pgm"));

  // cout << "Num configuration: " << numConfiguration << endl;
  // cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;
  //
  // cout << "------------------ HISTORY -----------------" << endl;
  // Calculate which cells have been visited only once
  list<Pose> tmp_history = utils.cleanHistory(&history, &record);
  utils.calculateDistance(tmp_history, &map, &astar );

  // cout << "------------------ TABULIST -----------------" << endl;
  utils.calculateDistance(tabuList, &map, &astar );
  double belief_accuracy = utils.findTags(&RFID_maps_list, &tags_coord, &map,
                  detection_log, accuracy_log, 
                  initRange, numConfiguration,
                  &rfid_tools);

  cout << "-----------------------------------------------------------------"<<endl;
  auto endMCDM = chrono::high_resolution_clock::now();
  content = to_string(w_info_gain) + ","  + to_string(w_travel_distance) + "," + to_string(w_sensing_time) + "," + to_string(w_rfid_gain) + "," + to_string(w_battery_status) + ","
            + to_string(norm_w_info_gain) + ","  + to_string(norm_w_travel_distance) + "," + to_string(norm_w_sensing_time) + "," + to_string(norm_w_rfid_gain) + "," + to_string(norm_w_battery_status) + ","
            + to_string(float(newSensedCells)/float(totalFreeCells)) + "," + to_string(numConfiguration) + ","
            + to_string(travelledDistance) + "," + to_string(totalScanTime) + "," + to_string(accumulated_received_power) + "," 
            + to_string(100*batteryTime/MAX_BATTERY) + ", " + to_string(belief_accuracy) + "\n";
  utils.filePutContents(out_log, content, true );

  double totalTimeMCDM = chrono::duration<double,milli> ( endMCDM -startMCDM ).count();
  cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, " << totalTimeMCDM/1000 <<" s, " <<
          totalTimeMCDM/60000 << " m "<< endl;

  utils.printResult(newSensedCells, totalFreeCells, precision, 
                    numConfiguration, travelledDistance, numOfTurning,
                    totalAngle, totalScanTime, accumulated_received_power, &batteryTime);

  //     cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, " << totalTimeMCDM/1000 <<" s, " <<
  // 		totalTimeMCDM/60000 << " m "<< endl;

}
