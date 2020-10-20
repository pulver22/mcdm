#include <algorithm>
#include <iostream>
#include <iterator>
#include "map.h"
#include "newray.h"
#include "mcdmfunction.h"
#include "evaluationrecords.h"
#include "Criteria/traveldistancecriterion.h"
#include "constants.h"
//#include "radio_models/propagationModel.cpp"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <ctime>
#include "utils.h"
// #include "RFIDGridmap.h"
#include "yaml-cpp/yaml.h"
#include "RadarModel.hpp"
#include <vector>
#include <random>



using namespace std;
using namespace dummy;
using namespace YAML;


// Example: ./mcdm_online_exploration ./../Images/cor_map_05_00_new1.pgm 1 99 99 180 5 180 1 0 1 54 143 865e6 0
int main ( int argc, char **argv )
{

  auto startMCDM = chrono::high_resolution_clock::now();

  int arguments = 23;
  if (argc<=arguments){
    std::cout << "Missing arguments! You provided "<< (argc-1)<< " and you need: "<< arguments << endl;
    return 0;
  } else {
    std::cout << "Call: " << endl << "\t" ;
    for (int i=0; i<argc;i++){
    std::cout << argv[i] << " ";  
    }    
    std::cout << endl << endl;  

    std::cout << "Arguments:" << endl;
    std::cout << "- Map image file: " << argv[1] << endl;
    std::cout << "- Map image resolution: " << argv[2] << " m./cell?" << endl;
    std::cout << "- initial Y-position of the robot in map frame: " << argv[3] << " cell?" << endl;
    std::cout << "- initial X-position of the robot in map frame: " << argv[4] << " cell?" << endl;
    std::cout << "- initial orientation of the robot in map frame: " << argv[5] << " (deg?) in steps of 15 degs?" << endl;
    std::cout << "- initRange: " << argv[6] << " cells?" << endl;
    std::cout << "- initial FOV of the robot sensor: " << argv[7] << " degs?" << endl;
    std::cout << "- precision: " << argv[8] << " -?" << endl;
    std::cout << "- threshold: " << argv[9] << " -?" << endl;

    std::cout << "- resolution to use for the planningGrid and RFIDGrid: " << argv[10] << " m./cell?" << endl;
    std::cout << "- YAML file with tag locations: " << argv[11] << endl;
    std::cout << "- freq: " << argv[12] << " Hertzs" << endl;
    std::cout << "- txtPower: " << argv[13] << " dBs" << endl;

    std::cout << "- w_info_gain: " << argv[14] << " -?" << endl;
    std::cout << "- w_travel_distance: " << argv[15] << " -?" << endl;
    std::cout << "- w_sensing_time: " << argv[16] << " -?" << endl;
    std::cout << "- w_rfid_gain: " << argv[17] << " -?" << endl;
    std::cout << "- w_battery_status: " << argv[18] << " -?" << endl;
    std::cout << "- out_log: " << argv[19] << endl;
    std::cout << "- coverage_log: " << argv[20] << endl;
    std::cout << "- out_log: " << argv[21] << endl;
    std::cout << "- ellipse X_min: " << argv[22] << endl;
    std::cout << "- accuracy_log: " << argv[23] << endl;
    std::cout << "- use_mcdm: " << argv[24] << endl;
  }





  ifstream infile;
  infile.open ( argv[1] );  // the path to the map
  double resolution = atof ( argv[2] );  // the resolution of the map
  double imgresolution = atof ( argv[10] );  // the resolution to use for the planningGrid and RFIDGrid
  dummy::Map map = dummy::Map ( infile,resolution, imgresolution );
  
  // std::cout << "Map dimension: " << map.getNumGridCols() << " : "<<  map.getNumGridRows() << endl;
  int gridToPathGridScale = map.getGridToPathGridScale();
  // i switched x and y because the map's orientation inside and outside programs are different
  long initX = static_cast<long>( atoi ( argv[4] ) *imgresolution );  // initial X-position of the robot in map frame
  long initY = static_cast<long>( atoi ( argv[3] ) *imgresolution );  // initial Y-position of the robot in map frame
  // std::std::cout << "initX: " << initX << " initY: " << initY << std::endl;
  int initOrientation = atoi ( argv[5] );  // initial orientation of the robot in map frame
  double initFov = atoi ( argv[7] );  // initial FOV of the robot sensor
  initFov = initFov * PI /180;
  int initRange = atoi ( argv[6] );
  double precision = atof ( argv[8] );
  double threshold = atof ( argv[9] );
  // RFID
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
  std::string entropy_log_path (argv[25]);
  string entropy_log = entropy_log_path + "/entropy.csv";
  bool use_mcdm = bool(atoi(argv[24]));
  //x,y,orientation,range,FOV
  double norm_w_info_gain, norm_w_travel_distance, norm_w_sensing_time, norm_w_rfid_gain, norm_w_battery_status;
  double sum_w = w_info_gain + w_travel_distance + w_sensing_time + w_rfid_gain + w_battery_status;
  norm_w_info_gain = w_info_gain / sum_w;
  norm_w_travel_distance = w_travel_distance / sum_w;
  norm_w_sensing_time = w_sensing_time / sum_w;
  norm_w_rfid_gain = w_rfid_gain / sum_w;
  norm_w_battery_status = w_battery_status / sum_w;
  Utilities utils(norm_w_info_gain, norm_w_travel_distance, norm_w_sensing_time, norm_w_rfid_gain, norm_w_battery_status);
  MCDMFunction function(norm_w_info_gain, norm_w_travel_distance, norm_w_sensing_time, norm_w_rfid_gain, norm_w_battery_status ,use_mcdm);
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
  list<Pose> tabuList;
  tabuList.push_back(target);
  list<Pose> nearCandidates;
  bool btMode = false;
  double totalAngle = 0;
  Astar astar;
  double totalScanTime = 0;
  bool act = true;
  int encodedKeyValue = 1;
  string content;

  double distStep = 0.1;
  double minX, maxX, minY, maxY;

  // Radar model: 
  double nx = 200*resolution; // radar model active area x-range m.
  double ny = 120*resolution;  // radar model active area y-range m.  
  double rs = resolution; // radar model grid resolution m./cell :: SAME AS INPUT IMAGE!!!
  double sigma_power = 3.92; //dB
  double sigma_phase = 1; //rads
  // txtPower = txtPower; // NOTE: Added for debug
  std::vector<double> freqs{ freq }; // only 1 freq... nice!
  // std::vector<double> freqs{ MIN_FREQ_NA,MIN_FREQ_NA+STEP_FREQ_NA,MIN_FREQ_NA+2.0*STEP_FREQ_NA }; 

  std::cout <<"Building radar model." << endl;
  RadarModel rm(rs, sigma_power, sigma_phase, txtPower, freqs, tags_coord, argv[1] );
  std::cout << "Radar model built." << endl;
  rm.PrintRefMapWithTags("/tmp/scenario.png");  

  long x, y = 0;
  int orientation, range;
  double FOV, major_axis, minor_axis, scanAngle, rxPower, phase;
  string encoding, actualPose;

  list<Pose> frontiers, tmp_history;
  bool break_loop;
  double accumulated_received_power = 0.0;
  double batteryTime = MAX_BATTERY;
  double batteryPercentage = 100;
  double distance = 0.0;
  double tmp_numOfTurning = 0.0;
  double translTime = 0.0;
  double rotTime = 0.0;

  RFID_tools rfid_tools;
  rfid_tools.rm = &rm;
  rfid_tools.tags_coord = tags_coord;
  rfid_tools.freq = freq;
  rfid_tools.txtPower = txtPower;
  rfid_tools.sensitivity = SENSITIVITY;
  rfid_tools.RFID_maps_list = &RFID_maps_list;
  double entropy_map = 0;
  bool emptyTabuList = false;
  int tabuListCount = MAX_TABULIST_COUNT;
  bool explorationCompleted = false;
  do{
    // // if (graph2.size() == 1 and count > 1) break;
    // if (sensedCells >= 0.95 * precision * totalFreeCells){
    //   emptyTabuList = true;
    //   explorationCompleted = true;
    // } 

    // if (emptyTabuList == true and explorationCompleted == true){
    //   tabuList.clear();
    //   emptyTabuList = false;
    // } 
    
    // if (emptyTabuList == false and explorationCompleted == true){
    //   if(tabuListCount >= 0){
    //     tabuListCount--;
    //     cout << "count: " << tabuListCount << endl;
    //   } else {
    //     tabuListCount = MAX_TABULIST_COUNT;
    //     cout << "   -----> reset" << endl;
    //   }
    // } 
    

    // If we are doing "forward" navigation towards cells never visited before
    if ( btMode == false )
    {
      entropy_map = rfid_tools.rm->getMapTotalEntropy(entropy_log_path);
      std::cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells << " ["<< 100*(float)newSensedCells/(float)totalFreeCells <<
      "%] - Battery: " << to_string(batteryPercentage) << 
      ", Entropy: " << entropy_map << endl;
      
      // std::cout <<"   Graph: " << graph2.size() << endl;
      travelledDistance = utils.calculateDistance(tabuList, &map, &astar );
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
      // Record current entropy map on the log
      content = to_string(entropy_map) + "\n";
      utils.filePutContents(entropy_log, content);
      x = target.getX();
      y = target.getY();
      orientation = target.getOrientation();
      range = target.getRange();
      FOV = target.getFOV();
      actualPose = function.getEncodedKey ( target,0 );
      map.setCurrentPose ( target );

      //NOTE; calculate path and turnings between actual position and goal
      // Calculate the distance between the previous robot pose and the next one (target)
      // string path = astar.pathFind ( target.getX(), target.getY(), previous.getX(), previous.getY(), &map );
      // // Update the overall covered distance
      // distance = astar.lengthPath(path);
      // travelledDistance = travelledDistance + distance;
      // tmp_numOfTurning = astar.getNumberOfTurning ( path );
      // // translTime = distance / TRANSL_SPEED;
      // // rotTime = tmp_numOfTurning / ROT_SPEED;
      // // // NOTE: in backtracking we are not phisically moving
      // // batteryTime = batteryTime - (translTime + rotTime);  
      // // Update the overall number of turnings
      // numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );

      encoding = to_string ( target.getX() ) + to_string ( target.getY() );
      visitedCell.emplace ( encoding,0 );
      // Get the sensing time required for scanning
      target.setScanAngles ( ray.getSensingTime ( &map,x,y,orientation,FOV,range ) );
      // Perform a scanning operation
      // utils.getEllipseSize(range, atoi ( argv[21] ), &major_axis, &minor_axis);
      double X_max = range;
      double X_min = atoi ( argv[22] );
      double focal_length = (X_max - X_min) / 2.0; // (X_max - X_min)/2
      major_axis = focal_length + X_min;  // (focal_length + X_min)
      minor_axis = sqrt(pow(major_axis, 2) - pow(focal_length, 2));
      newSensedCells = sensedCells + ray.performSensingOperationEllipse ( &map,x,y,orientation, target.getScanAngles().first, target.getScanAngles().second, major_axis, minor_axis);
      // Calculate the scanning angle
      scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      // Update the overall scanning time
      totalScanTime += utils.calculateScanTime ( scanAngle*180/PI );
      // Update bot the PP and the RFID maps
      utils.updateMaps(&map, &target, &rfid_tools, false);
        rfid_tools.rm->saveProbMapDebug("/tmp/test/",0,count,x,y,-orientation*M_PI/180);
      // Search for new candidate position
      ray.findCandidatePositions ( &map,x,y,orientation,FOV,range );
      vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();

      // Reduce number of candidate position
      // std::random_device rd;
      // std::mt19937 g(rd());
      // std::shuffle(candidatePosition.begin(), candidatePosition.end(), g);
      // cout << "Candidates: " << candidatePosition.size() << endl;
      // if (candidatePosition.size() > 15){
      //   while (candidatePosition.size() > 15){
      //     candidatePosition.pop_back();
      //   }
      // }
      

      ray.emptyCandidatePositions();
      // Push position into the navigation graph
      break_loop = utils.updateNavigationGraph(&count, &function, &graph2, &target, &map, &x, &y,
                                              &orientation, &range, &FOV, &threshold, &actualPose, &rfid_tools, &batteryTime, &explorationCompleted);
      if (break_loop == true) break;
      break_loop = utils.forwardMotion(&target, &previous, &frontiers, &nearCandidates, &candidatePosition, &ray, &map,
                                            &x, &y, &orientation, &FOV, &range, &graph2,
                                            &record, &function, &threshold, &count, &history,
                                            &tmp_history, &tabuList, &astar, &imgresolution, &travelledDistance,
                                            &sensedCells, &newSensedCells, &totalFreeCells, &totalScanTime, &out_log, 
                                            &numConfiguration, &actualPose, &encodedKeyValue, &totalAngle, &numOfTurning,
                                            &scanAngle, &btMode, &rfid_tools, &accumulated_received_power, &precision, &batteryTime, &explorationCompleted);
      // cout << "break_loop: " << break_loop << endl;                                      
      // calculate the accumulate received power
      for (int tag_id = 0; tag_id < tags_coord.size(); tag_id++){
        // mfc: previous
        //double rx_power = rfid_tools.rm->received_power_friis(tags_coord[tag_id].first, tags_coord[tag_id].second, freq, txtPower);
        double rx_power = rfid_tools.rm->received_power_friis_with_obstacles(target.getX(), target.getY(), target.getOrientation() * PI/180.0,tags_coord[tag_id].first, tags_coord[tag_id].second, 0, freq);
        //mfc: the above gets the received power between a robot in "target" in METERS and tags_coord[i] in METERS. I'm assuming orientation is in deg.
        accumulated_received_power += rx_power;
      }
      // cout << "here" << endl;
      
      if (break_loop == true) break;

    }
    // ... otherwise, if we are doing backtracking
    else{ 
      // std::cout << "   Graph: " << graph2.size() << endl;
      // cout << "   nearCandidates: " << nearCandidates.size() << endl;
      // if (graph2.size() == 1) break;

      x = target.getX();
      y = target.getY();
      orientation = target.getOrientation();
      range = target.getRange();
      FOV = target.getFOV();
      actualPose = function.getEncodedKey ( target,0 );
      map.setCurrentPose ( target );
      //NOTE; calculate path and turnings between actual position and goal
      // Calculate the distance between the previous robot pose and the next one (target)
      // string path = astar.pathFind ( target.getX(), target.getY(), previous.getX(), previous.getY(), &map );
      // Update the overall covered distance
      // distance = astar.lengthPath(path);
      // travelledDistance = travelledDistance + distance;
      // tmp_numOfTurning = astar.getNumberOfTurning ( path );
      // translTime = distance / TRANSL_SPEED;
      // rotTime = tmp_numOfTurning / ROT_SPEED;
      // NOTE: in backtracking we are not phisically moving
      // batteryTime = batteryTime - (translTime + rotTime);  
      // Update the overall number of turnings
      // numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );
      encoding = to_string ( target.getX() ) + to_string ( target.getY() );
      visitedCell.emplace ( encoding,0 );
      // Set the previous cell to be the same of the current one
      previous = target;
      // Calculate how much time it takes to scan the current area
      target.setScanAngles ( ray.getSensingTime ( &map,x,y,orientation,FOV,range ) );
      // Get the scanning angle
      scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      // Update the overall scanned angle
      totalAngle += scanAngle;
      // ...and the overall scan time
      totalScanTime += utils.calculateScanTime ( scanAngle*180/PI );
      // Update bot the PP and the RFID maps
      // std::cout << "Here" << endl;
      // utils.updateMaps(&map, &target, &rfid_tools, false);
      // std::cout << "Here" << endl;
      // Remove the current pose from the list of possible candidate cells
      utils.cleanPossibleDestination2 ( &nearCandidates,target );
      // Get the list of the candidate cells with their evaluation
      EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates, &map, threshold, &rfid_tools, &batteryTime, &explorationCompleted );
      // std::cout << "   record: " << record->size() << endl;
      // If there are candidate cells
      if ( record->size() != 0 )
      {
        utils.recordContainsCandidatesBT(record,
                                    &count, &target, &previous, &actualPose, &nearCandidates, 
                                    &graph2, &map, &function, &tabuList, &history, 
                                    &encodedKeyValue, &astar, &numConfiguration,
                                    &totalAngle, &travelledDistance, &numOfTurning , &scanAngle, 
                                    &btMode, &threshold, &rfid_tools, &batteryTime, &explorationCompleted);
      }
      // ... if there are not candidate cells
      else
      {
        utils.recordNOTContainsCandidatesBT(record, 
                                    &count, &target, &previous, &actualPose, &nearCandidates, 
                                    &graph2, &map, &function, &tabuList, &history, 
                                    &encodedKeyValue, &astar, &numConfiguration,
                                    &totalAngle, &travelledDistance, &numOfTurning , &scanAngle, 
                                    &btMode, &threshold, &batteryTime);
      }
      delete record;
    }
    
    // batteryPercentage = utils.calculateRemainingBatteryPercentage(tabuList, &map, &astar);
    batteryPercentage = 100*batteryTime/MAX_BATTERY;
  }
  // Perform exploration until a certain coverage is achieved
  while ( sensedCells < precision * totalFreeCells and batteryPercentage > 0.0);
  explorationCompleted = true;
  // Add the last position to history list
  history.push_back(function.getEncodedKey(target, 1));
  list<Pose> nextNodes;
  tabuList.clear();
  vector<string>::iterator it_history = history.begin();
  Pose tmp;
  // Build a graph modeled as hashmap. Keys are individual nodes, and attributes are
  // all the cells reachable from the keys within a certain distance (e.g. 10 meters)
  std::cout << "Preparing topological map..." << endl;
  std::vector<pair<Pose, list<Pose>>> topologicalMap = utils.getTopologicalMap(&astar, &record, &history, &map);
  // std::cout << "TopologicalMap nodes: " << topologicalMap.size() << endl;
  // std::cout << topologicalMap[0].first.getX() << endl;
  // for (it_history; it_history!=prev(history.end()); it_history++){
  //     tmp = record.getPoseFromEncoding(*it_history);
  //     nextNodes.push_back(tmp);
  //   }


  do{
    /**
     * Get current pose in the history tree
     * Select N previous and N following positions
     * Evaluate them
     * Pick the best and go there
     * Iterate
     */
    
    // Calculate the scanning angle
    scanAngle = target.getScanAngles().second - target.getScanAngles().first;
    // Update the overall scanning time
    totalScanTime += utils.calculateScanTime ( scanAngle*180/PI );
    // Update bot the PP and the RFID maps
    utils.updateMaps(&map, &target, &rfid_tools, false);
      

    entropy_map = rfid_tools.rm->getMapTotalEntropy(entropy_log_path);
    batteryPercentage = 100*batteryTime/MAX_BATTERY;
      std::cout << "\n[ENTROPY]Area sensed: " << newSensedCells << " / " << totalFreeCells << " ["<< 100*(float)newSensedCells/(float)totalFreeCells <<
      "%] - Battery: " << to_string(batteryPercentage) << 
      ", Entropy: " << entropy_map << endl;
    // Record current map's entropy on the log
    content = to_string(entropy_map) + "\n";
    utils.filePutContents(entropy_log, content, true );
    
    // TODO: I am not sure this is required anymore with the topological map
    if(tabuListCount >= 0){
        tabuListCount--;
      } else {
        tabuList.clear();
        tabuListCount = MAX_TABULIST_COUNT;
      }
    // nextNodes.clear();
    record.clear();
    // cout << "nextNodes: " << nextNodes.size() << endl;
    
    
    // string current_pose = function.getEncodedKey(target, 2);
    // while(record.size() == 0){
    //   for ( it_history; it_history!=prev(history.end(),1); it_history++)
    //   {
    //     if ((*it_history).back() != '2'){
    //       tmp = record.getPoseFromEncoding(*it_history);
    //       // cout << "tmp: " << *it_history << endl;  
    //       if (tmp.isEqual(target))
    //       {
    //         // cout << "\nFound" << endl;
    //         vector<string>::iterator next_it = it_history;//next(it_history, 1);
    //         vector<string>::iterator prev_it = it_history;//prev(it_history, 1);
    //         for (int i=1; i<10; i++){
    //           ++next_it;
    //           --prev_it;
    //           if (next_it < history.end()){
    //             tmp = record.getPoseFromEncoding(*next_it);        
    //             if (!tmp.isEqual(target) and !utils.contains(tabuList, tmp)){
    //                 nextNodes.push_back(tmp);
    //                 // cout << "[n] " << *next_it << endl;
    //             }    
    //           }
    //           if (prev_it > history.begin()){
    //             tmp = record.getPoseFromEncoding(*prev_it);          
    //             if (!tmp.isEqual(target) and !utils.contains(tabuList, tmp)){
    //               // cout << "[p] " << *prev_it << endl;
    //               nextNodes.push_back(tmp);
    //             }  
    //           }          
    //         }
    //         break;  // to prevent the same cell is counted for multiple times
    //       }
    //     }
    //     // TODO: code loops here while record is equal to 0
    //   }
      // cout << "nextNodes: " << nextNodes.size() << endl;
    while(record.size() == 0){
      // Find current position in the topological map, retrieve the neighbors
      nearCandidates.clear();
      for (int i=0; i < topologicalMap.size(); i++){
        Pose tmp = topologicalMap[i].first;
        if (tmp.isEqual(target)){
          nearCandidates = topologicalMap[i].second;
          // std::cout << "Candidates: " << nearCandidates.size() << endl;
        } 
      }
      if (nearCandidates.size() == 0){
        std::cout << "[ERROR] in finding new positions" << endl;
        break;
      }
      
      frontiers = nearCandidates;
      list<Pose>::iterator it_tabuList;
      cout << "nearCandidates: " << nearCandidates.size() << endl;
      for (it_tabuList=tabuList.begin(); it_tabuList!=tabuList.end(); it_tabuList++){
        // nearCandidates.remove(*it_tabuList);
        std::list<Pose>::iterator findIter = std::find(
          nearCandidates.begin(), nearCandidates.end(), *it_tabuList);
        if (findIter != nearCandidates.end()) {
          nearCandidates.erase(findIter);
        }
      }
      cout << "nearCandidates: " << nearCandidates.size() << endl;

      // Evaluate all the candidate position
      record = *function.evaluateFrontiers(nearCandidates, &map, threshold, &rfid_tools, &batteryTime, &explorationCompleted);
      cout << "Record1: " << record.size() << endl; 
      // Clean all the possible destination from cells recently visited
      for (list<Pose>::iterator it = tabuList.begin(); it != tabuList.end();
          it++) {
        // utils.cleanPossibleDestination2(&nearCandidates, *it);
        record.removeFrontier(*it);
      }
      
      cout << "Record2: " << record.size() << endl; 

      if (record.size() == 0){
        std::cout << "F6" << endl;
        tabuList.push_back(target);
        // break_loop = utils.recordNOTContainsCandidates(&graph2, &record, &target,
        //                         &previous, &history, &function, &count);
        // if (break_loop == true) break;

        // Select a random neighbout as new target and look new candidates from there
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0, frontiers.size()-1); // guaranteed unbiased

        int random_integer = uni(rng);
        list<Pose>::iterator it_candidates = frontiers.begin();
        std::advance(it_candidates, random_integer);
        target = *(it_candidates);
      }
    }

    // cout << "Record1: " << record.size() << endl; 
    //   // Clean all the possible destination from cells recently visited
    //   for (list<Pose>::iterator it = tabuList.begin(); it != tabuList.end();
    //       it++) {
    //     // utils.cleanPossibleDestination2(&nearCandidates, *it);
    //     record.removeFrontier(*it);
    //   }
      
    string current_pose = function.getEncodedKey(target, 2);
    cout << "CurrentPose: " << current_pose << endl;
    rfid_tools.rm->saveProbMapDebug("/tmp/test/",0,count,target.getX(),target.getY(),-target.getOrientation()*M_PI/180);
    // cout << "Record2: " << record.size() << endl; 
    nearCandidates = record.getFrontiers();
    // // Clean all the possible destination from cells recently visited
    // for (list<Pose>::iterator it=tabuList.begin(); it!=tabuList.end(); it++){
    //   // utils.cleanPossibleDestination2(&nearCandidates, *it);
    //   record.removeFrontier(*it);
    // }

    if ( record.size() != 0 ){
      std::cout <<"F5" << endl;
      break_loop = utils.recordContainsCandidates( &record, &count, &target, &previous, 
                                &actualPose, &nearCandidates, &graph2, &map, &function, &tabuList, 
                                &history, &encodedKeyValue, &astar , &numConfiguration, &totalAngle,
                                &travelledDistance, &numOfTurning , &scanAngle, &btMode, &threshold, 
                                &rfid_tools, &batteryTime, &explorationCompleted);
      // std::cout << "Break: " << break_loop << endl;
      if (break_loop == true) break;
    }
    // ... otherwise, if there are no candidate positions
    // else
    // {
    //   std::cout <<"F6" << endl;
    //   break_loop = utils.recordNOTContainsCandidates(&graph2, &record, &target,
    //                             &previous, &history, &function, &count);
    //   if (break_loop == true) break;
    // }
    // calculate the accumulate received power
    for (int tag_id = 0; tag_id < tags_coord.size(); tag_id++){
      // mfc: previous
      //double rx_power = rfid_tools.rm->received_power_friis(tags_coord[tag_id].first, tags_coord[tag_id].second, freq, txtPower);
      double rx_power = rfid_tools.rm->received_power_friis_with_obstacles(target.getX(), target.getY(), target.getOrientation() * PI/180.0,tags_coord[tag_id].first, tags_coord[tag_id].second, 0, freq);
      //mfc: the above gets the received power between a robot in "target" in METERS and tags_coord[i] in METERS. I'm assuming orientation is in deg.
      accumulated_received_power += rx_power;
    }
  }
  while( entropy_map > 20.0 and batteryPercentage > 2.0);
  cout << "Out" << endl;
  // Plotting utilities
  // map.drawVisitedCells ();
  // map.printVisitedCells ( history );
  // map.drawRFIDScan();
  // map.drawRFIDGridScan(myGrid1);
  // utils.saveRFIDMaps(&RFID_maps_list, "/tmp/");
  // utils.saveRFIDMapsWithGroundTruths(&RFID_maps_list, &tags_coord, "/tmp/D");
  // std::cout << "------------------ HISTORY -----------------" << endl;
  // Calculate which cells have been visited only once
  // FIXME: history doesn't contain last visited cell
  tmp_history = utils.cleanHistory(&history, &record);
  // cout << "1" << endl;
  travelledDistance = utils.calculateDistance(tmp_history, &map, &astar );
  // cout << "2" << endl;

  // std::cout << "------------------ TABULIST -----------------" << endl;
  // NOTE: tabuList is the most reliable source of information regarding the cells actually visited
  // because it's is filled only when the cells are visited for the first time and not during virtual
  // backtracking. So we use tabuList for calculating the final "real" traversedDistance and remainingBatteryTime
  travelledDistance =  utils.calculateDistance(tabuList, &map, &astar );
  batteryPercentage = utils.calculateRemainingBatteryPercentage(tabuList, &map, &astar);
  // cout << "3" << endl;

  double belief_accuracy = utils.findTags(&RFID_maps_list, &tags_coord, &map,
                  detection_log, accuracy_log, 
                  initRange, numConfiguration,
                  &rfid_tools);
  std::cout << "-----------------------------------------------------------------"<<endl;
  auto endMCDM = chrono::high_resolution_clock::now();
  content = to_string(w_info_gain) + ","  + to_string(w_travel_distance) + "," + to_string(w_sensing_time) + "," + to_string(w_rfid_gain) + "," + to_string(w_battery_status) + ","
            + to_string(norm_w_info_gain) + ","  + to_string(norm_w_travel_distance) + "," + to_string(norm_w_sensing_time) + "," + to_string(norm_w_rfid_gain) + "," + to_string(norm_w_battery_status) + ","
            + to_string(float(newSensedCells)/float(totalFreeCells)) + "," + to_string(numConfiguration) + ","
            + to_string(travelledDistance) + "," + to_string(totalScanTime) + "," + to_string(accumulated_received_power) + "," 
            + to_string(batteryPercentage) + ", " + to_string(belief_accuracy) + "\n";
  utils.filePutContents(out_log, content, true );

  double totalTimeMCDM = chrono::duration<double,milli> ( endMCDM -startMCDM ).count();
  std::cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, " << totalTimeMCDM/1000 <<" s, " <<
          totalTimeMCDM/60000 << " m "<< endl;

  utils.printResult(newSensedCells, totalFreeCells, precision, 
                    numConfiguration, travelledDistance, numOfTurning,
                    totalAngle, totalScanTime, accumulated_received_power, &batteryPercentage);

  // std::cout << "Saving tag distribution maps... "<< endl;
  rfid_tools.rm->saveProbMaps("/tmp/");

  std::cout << "Saving debug distribution maps... "<< endl;
  // rm->normalizeRFIDMap();
  // for each tag:

  for (int t = 0; t < tags_coord.size(); t++){
    // std::cout << "---[" << t <<"]----------------" << endl;
    rfid_tools.rm->saveProbMapDebug("/tmp/",t,count,x,y,orientation);
  }
  cout << "Final entropy: " << entropy_map << endl;
}
