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

Pose getRandomFrontier(list<Pose> nearCandidates);

int main ( int argc, char **argv )
{
  auto startMCDM = chrono::high_resolution_clock::now();
  ifstream infile;
  infile.open ( argv[1] );  // the path to the map
  double resolution = atof ( argv[2] );  // the resolution of the map
  double imgresolution = atof ( argv[10] );  // the resolution to use for the planningGrid and RFIDGrid
  dummy::Map map = dummy::Map ( infile,resolution, imgresolution );
  cout << "Map dimension: " << map.getNumGridCols() << " : "<<  map.getNumGridRows() << endl;
  int gridToPathGridScale = map.getGridToPathGridScale();
  // i switched x and y because the map's orientation inside and outside programs are different
  long initX = static_cast<long>( atoi ( argv[4] ) *imgresolution );  // initial X-position of the robot in map frame
  long initY = static_cast<long>( atoi ( argv[3] ) *imgresolution );  // initial Y-position of the robot in map frame
  int initOrientation = atoi ( argv[5] );  // initial orientation of the robot in map frame
  double initFov = atoi ( argv[7] ) * PI /180;  // initial FOV of the robot sensor
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
  // double absTag1_X = 37 * resolution;
  // double absTag1_Y = 58 * resolution;

  // double absTag2_X = 44 * resolution;
  // double absTag2_Y = 31 * resolution;
  
  // double absTag3_X = 130 * resolution;
  // double absTag3_Y = 45 * resolution;
  
  // double absTag4_X = 150 * resolution;
  // double absTag4_Y = 108 * resolution;

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


  double distStep = 0.1;
  double minX, maxX, minY, maxY;

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
  RadarModel rm(rs, sigma_power, sigma_phase, txtPower, freqs, tags_coord, argv[1] );
  //RadarModel rm(nx, ny, rs, sigma_power, sigma_phase, txtPower, freqs, tags_coord, argv[1] );
  cout << "Radar model built." << endl;
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

  do
  {
    // If we are doing "forward" navigation towards cells never visited before
    if ( btMode == false )
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
      encoding = to_string ( target.getX() ) + to_string ( target.getY() );
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
      utils.updateMaps(&map, &target, &rfid_tools, false);
      // Search for new candidate position
      ray.findCandidatePositions ( &map,x,y,orientation,FOV,range );
      vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
      ray.emptyCandidatePositions();
      // Calculate the accumulated received power
      for (int tag_id = 0; tag_id < tags_coord.size(); tag_id++){
        // mfc: previous
        //double rx_power = rfid_tools.rm->received_power_friis(tags_coord[tag_id].first, tags_coord[tag_id].second, freq, txtPower);
        double rx_power = rfid_tools.rm->received_power_friis_with_obstacles(target.getX(), target.getY(), target.getOrientation() * PI/180.0,tags_coord[tag_id].first, tags_coord[tag_id].second, 0, freq);
        //mfc: the above gets the received power between a robot in "target" in METERS and tags_coord[i] in METERS. I'm assuming orientation is in deg.
        accumulated_received_power += rx_power;
      }
      // If the exploration just started
      if ( count == 0 )
      {
        // Calculate other three pose given the strating one
        string invertedPose = function.getEncodedKey ( invertedInitial,0 );
        string eastPose = function.getEncodedKey ( eastInitial,0 );
        string westPose = function.getEncodedKey ( westInitial,0 );
        list<Pose> empty ;
        std::pair<string,list<Pose>> pair1 = make_pair ( invertedPose,empty );
        std::pair<string,list<Pose>> pair2 = make_pair ( eastPose,empty );
        std::pair<string,list<Pose>> pair3 = make_pair ( westPose,empty );
        // And add them (with empty candidates) to the graph structure
        graph2.push_back ( pair1 );
        graph2.push_back ( pair2 );
        graph2.push_back ( pair3 );
      }

      // If it's not the first step but we are in one of the initial position (we come back here with backtracking)
      if ( count != 0 && ( target.isEqual ( invertedInitial ) || target.isEqual ( eastInitial ) || target.isEqual ( westInitial ) ) )
      {
        // If there are no more destination in the graph, terminates the navigation
        if ( graph2.size() == 0 ) break;
        graph2.pop_back();
        actualPose = function.getEncodedKey ( target,0 );
        // Add to the graph the initial positions and the candidates from there (calculated inside the function)
        utils.pushInitialPositions ( &map, x, y,orientation, range,FOV, threshold, actualPose, &graph2, &function, &rfid_tools, &batteryTime );
      }


      // If there are no new candidate positions from the current pose of the robot
      if ( candidatePosition.size() == 0 )
      {
        // Find candidates
        ray.findCandidatePositions2 ( &map,x,y,orientation,FOV,range );
        candidatePosition = ray.getCandidatePositions();
        ray.emptyCandidatePositions();

        // cout << "No other candidate position" << endl;
        // cout << "----- BACKTRACKING -----" << endl;
        // If the graph contains cells that can be explored
        if ( graph2.size() > 1 )
        {
          // Get the last position in the graph and then remove it
          string targetString = graph2.at ( graph2.size()-1 ).first;
          graph2.pop_back();
//          EvaluationRecords record;
          target = record.getPoseFromEncoding ( targetString );
          // Add it to the history as cell visited more than once
          history.push_back ( function.getEncodedKey ( target,2 ) );
          // cout << "[BT]No significative position reachable. Come back to previous position" << endl;
          count = count + 1;
        }
        //...otherwise, if the graph does not contain cells that can be explored
        // The navigation is finished!
        else
        {
          cout << "Num configuration: " << numConfiguration << endl;
          cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;
          cout << "------------------ HISTORY -----------------" << endl;
          // Retrieve the cell visited only the first time
          list<Pose> tmp_history = utils.cleanHistory(&history, &record);
          utils.calculateDistance(tmp_history, &map, &astar );

          cout << "------------------ TABULIST -----------------" << endl;
          // Calculate the path connecting the cells in the tabulist, namely the cells that are visited one time and couldn't be visite again
          utils.calculateDistance(tabuList, &map, &astar );

          // Normalise the travel distance in meter
          // NOTE: assuming that the robot is moving at 0.5m/s and the resolution of the map is 0.5m per cell)
          if ( imgresolution == 1.0 )
          {
            travelledDistance = travelledDistance/2;
          }
          utils.printResult(newSensedCells, totalFreeCells, precision, numConfiguration, travelledDistance, numOfTurning,
              totalAngle, totalScanTime, accumulated_received_power, &batteryTime);
          content = to_string(w_info_gain) + ","  + to_string(w_travel_distance) + "," + to_string(w_sensing_time) + "," + to_string(w_rfid_gain) + ","
                           + to_string(float(newSensedCells)/float(totalFreeCells)) + "," + to_string(numConfiguration) + ","
                           + to_string(travelledDistance) + "," + to_string(totalScanTime) + "\n";
          utils.filePutContents(out_log, content, true );
          exit ( 0 );
        }

        sensedCells = newSensedCells;

      }
      //... otherwise, if there are further candidate new position from the current pose of the robot
      else
      {
        // need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
        list<Pose> frontiers;
        // For every candidate positio, create 8 pose with a different orientation each and consider them as frontiers
        vector<pair<long,long> >::iterator it =candidatePosition.begin();
        for ( it; it != candidatePosition.end(); it++ )
        {
          Pose p1 = Pose ( ( *it ).first, ( *it ).second,0 ,range,FOV );
          Pose p2 = Pose ( ( *it ).first, ( *it ).second,45,range,FOV );
          Pose p3 = Pose ( ( *it ).first, ( *it ).second,90,range,FOV );
          Pose p4 = Pose ( ( *it ).first, ( *it ).second,135,range,FOV );
          Pose p5 = Pose ( ( *it ).first, ( *it ).second,180,range,FOV );
          Pose p6 = Pose ( ( *it ).first, ( *it ).second,225,range,FOV );
          Pose p7 = Pose ( ( *it ).first, ( *it ).second,270,range,FOV );
          Pose p8 = Pose ( ( *it ).first, ( *it ).second,315,range,FOV );
          frontiers.push_back ( p1 );
          frontiers.push_back(p2);
          frontiers.push_back ( p3 );
          frontiers.push_back(p4);
          frontiers.push_back ( p5 );
          frontiers.push_back(p6);
          frontiers.push_back ( p7 );
          frontiers.push_back(p8);

        }

        unexploredFrontiers = frontiers;

        // Evaluate the frontiers and return a list of <frontier, evaluation> pairs
        EvaluationRecords *record = function.evaluateFrontiers ( frontiers, &map, threshold, &rfid_tools, &batteryTime );
        nearCandidates = record->getFrontiers();

        // If there are candidate positions
        if ( record->size() != 0 )
        {
          // Set the previous pose equal to the current one (represented by target)
          previous = target;
          target = getRandomFrontier(nearCandidates);
          // Select the new robot destination from the list of candidates
          // std::pair<Pose,double> result = function.selectNewPose ( record );
          // target = result.first;
          // If the selected destination does not appear among the cells already visited
          if ( ! utils.contains ( tabuList,target ))
          {
            act = true;
            // Add it to the list of visited cells as first-view
            encodedKeyValue = 1;
            utils.updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle, &batteryTime);
          }
          // ...otherwise, if the seleced cell has already been visited
          else
          {
            // If the graph is empty, stop the navigation
            if ( graph2.size() == 0 ) break;
            // If there still are more candidates to explore from the last pose in the graph
            if ( graph2.at ( graph2.size()-1 ).second.size() != 0 )
            {
              // cout << "[BT1 - Tabulist]There are visible cells but the selected one is already explored!Come back to second best position from the previous position"<< endl;
              // Remove the current position from possible candidates
              utils.cleanPossibleDestination2 ( &nearCandidates, target );
              // Get the list of new candidate position with associated evaluation
              record = function.evaluateFrontiers ( nearCandidates, &map, threshold, &rfid_tools, &batteryTime );
              // If there are candidate positions
              if ( record->size() != 0 )
              {
                // Select the new pose of the robot
                target = getRandomFrontier(nearCandidates);
                // std::pair<Pose,double> result = function.selectNewPose ( record );
                // target = result.first;
                encodedKeyValue = 1;
                utils.updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle, &batteryTime);
                // Set that we are now in backtracking
                btMode = true;
              }
              // If there are no more candidate position from the last position in the graph
              else
              {
                // if the graph is now empty, stop the navigation
                if ( graph2.size() == 0 ) break;
                // Otherwise, select as new position the last cell in the graph and then remove it from there
                string targetString = graph2.at ( graph2.size()-1 ).first;
                graph2.pop_back();
                target = record->getPoseFromEncoding ( targetString );
              }
            }
            // ... if the graph still does not present anymore candidate positions for its last pose
            else
            {
              // Remove the last element (cell and associated candidate from there) from the graph
              graph2.pop_back();
              // Select as new target, the new last element of the graph
              string targetString = graph2.at ( graph2.size()-1 ).first;
              target = record->getPoseFromEncoding ( targetString );
              // Save it history as cell visited more than once
              history.push_back ( function.getEncodedKey ( target,2 ) );
              // cout << "[BT2 - Tabulist]There are visible cells but the selected one is already explored!Come back to two position ago"<< endl;
              count = count + 1;
            }

          }
        }
        // ... otherwise, if there are no candidate positions
        else
        {
          // If the graph is empty, stop the navigation
          if ( graph2.size() == 0 ) break;
          // Select as new target the last one in the graph structure
          string targetString = graph2.at ( graph2.size()-1 ).first;
          // Remove it from the graph
          graph2.pop_back();
          target = record->getPoseFromEncoding ( targetString );
          // Check if the selected cell in the graph is the previous robot position
          if ( !target.isEqual ( previous ) )
          {
            // if it's not, set the old position as the current one
            previous = target;  //TODO: WHY?
            // cout << "[BT3]There are no visible cells so come back to previous position in the graph structure" << endl;
            // Save the new target in the history as cell visited more than once
            history.push_back ( function.getEncodedKey ( target,2 ) );
            count = count + 1;
          }
          // If the selected cell is the old robot position
          else
          {
            // If there are no more cells in the graph, just finish the navigation
            if ( graph2.size() == 0 )
            {
              // cout << "[BT4]No other possibilities to do backtracking on previous positions" << endl;
              break;
            }
            // Select the last position in the graph
            string targetString = graph2.at ( graph2.size()-1 ).first;
            // and remove it from the graph
            graph2.pop_back();
            target = record->getPoseFromEncoding ( targetString );
            // Set the previous pose as the current one
            previous = target;
            // cout << "[BT5]There are no visible cells so come back to previous position" << endl;
            // cout << "[BT5]Cell already explored!Come back to previous position"<< endl;
            // Add it in history as cell visited more than once
            history.push_back ( function.getEncodedKey ( target,2 ) );
            count = count + 1;
          }

        }



        //NOTE: not requested for testing purpose
        //usleep(microseconds);
        sensedCells = newSensedCells;
        frontiers.clear();
        candidatePosition.clear();
        delete record;
      }

    }
    // ... otherwise, if we are doing backtracking
    else
    {
      long x = target.getX();
      long y = target.getY();
      int orientation = target.getOrientation();
      int range = target.getRange();
      double FOV = target.getFOV();
      string actualPose = function.getEncodedKey ( target,0 );
      map.setCurrentPose ( target );
      //NOTE; calculate path and turnings between actual position and goal
      // cout<< function.getEncodedKey ( target,1 ) << endl;
      // Calculate the distance between the previous robot pose and the next one (target)
      string path = astar.pathFind ( target.getX(), target.getY(), previous.getX(), previous.getY(), &map );
      // Update the overall covered distance
      distance = astar.lengthPath(path);
      travelledDistance = travelledDistance + distance;
      tmp_numOfTurning = astar.getNumberOfTurning ( path );
      translTime = distance / TRANSL_SPEED;
      rotTime = tmp_numOfTurning / ROT_SPEED;
      batteryTime = batteryTime - (translTime + rotTime);
      // Update the overall number of turnings
      numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );
      string encoding = to_string ( target.getX() ) + to_string ( target.getY() );
      visitedCell.emplace ( encoding,0 );
      // Set the previous cell to be the same of the current one
      previous = target;

      // Calculate how much time it takes to scan the current area
      target.setScanAngles ( ray.getSensingTime ( &map,x,y,orientation,FOV,range ) );
      // Get the scanning angle
      double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      // Update the overall scanned angle
      totalAngle += scanAngle;
      // ...and the overall scan time
      totalScanTime += utils.calculateScanTime ( scanAngle*180/PI );
      // Update bot the PP and the RFID maps
      utils.updateMaps(&map, &target, &rfid_tools, false);
      // Remove the current pose from the list of possible candidate cells
      utils.cleanPossibleDestination2 ( &nearCandidates,target );
      // Get the list of the candidate cells with their evaluation
      EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates, &map, threshold, &rfid_tools, &batteryTime );

      // If there are candidate cells
      if ( record->size() != 0 )
      {
        // Find the new destination
        // std::pair<Pose,double> result = function.selectNewPose ( record );
        // target = result.first;
        target = getRandomFrontier(nearCandidates);
        // If this cells has not been visited before
        if ( ! utils.contains ( tabuList,target ) )
        {
          // Add it to the list of visited cells as first-view
          encodedKeyValue = 1;
          utils.updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle, &batteryTime);
          // Leave the backtracking branch
          btMode = false;
          nearCandidates.clear();
          // cout << "[BT-MODE4] Go back to previous positions in the graph" << endl;
        }
        // ... otherwise, if the cells has already been visisted
        else
        {
          // If there are other candidates
          if ( nearCandidates.size() != 0 )
          {
            // cout << "[BT-MODE1]Already visited, but there are other candidates" << endl;

            // Remove the destination from the candidate list
            utils.cleanPossibleDestination2 ( &nearCandidates,target );
            // Get the candidates with their evaluation
            EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates, &map, threshold , &rfid_tools, &batteryTime);
            // Select the new destination
            // std::pair<Pose,double> result = function.selectNewPose ( record );
            // target = result.first;
            target = getRandomFrontier(nearCandidates);

            // Add it to the list of visited cells as first-view
            encodedKeyValue = 1;
            utils.updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle, &batteryTime);
          }
          // ...otherwise, if there are no more candidates
          else
          {
            // cout << "[BT-MODE2] Go back to previous positions in the graph" << endl;
            // Select as target the last element in the graph
            string targetString = graph2.at ( graph2.size()-1 ).first;
            // And remove from the graph
            graph2.pop_back();
            target = record->getPoseFromEncoding ( targetString );
            // Add it to the history of cell as already more than once
            encodedKeyValue = 2;
            utils.updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle, &batteryTime);
            // Leave backtracking
            btMode = false;
            // Clear candidate list
            nearCandidates.clear();
          }
        }
      }
      // ... if there are not candidate cells
      else
      {
        // Select as new pose, the last cell in the graph
        string targetString = graph2.at ( graph2.size()-1 ).first;
        // and the remove it form the graph
        graph2.pop_back();
        target = record->getPoseFromEncoding ( targetString );

        // Add it in history as cell visited more than once
        encodedKeyValue = 2;
        utils.updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle, &batteryTime);
        // Leave backtracking
        btMode = false;
        // cout << "[BT-MODE3] Go back to previous positions in the graph" << endl;
      }
      delete record;
    }
    double batteryPercentage = utils.calculateRemainingBatteryPercentage(tabuList, &map, &astar);
  }
  // Perform exploration until a certain coverage is achieved
  while ( sensedCells < precision * totalFreeCells and batteryPercentage > 0.0);
  // Plotting utilities
  // map.drawVisitedCells ();
  // map.printVisitedCells ( history );
  // map.drawRFIDScan();
  // map.drawRFIDGridScan(myGrid);
  // myGrid.saveAs(("/home/pulver/Desktop/MCDM/rfid_result_gridmap.pgm"));

  // cout << "Num configuration: " << numConfiguration << endl;
  // cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;
 
  // tmp_history = utils.cleanHistory(&history, &record);
  // utils.calculateDistance(tmp_history, &map, &astar );

  // cout << "------------------ TABULIST -----------------" << endl;
  travelledDistance =  utils.calculateDistance(tabuList, &map, &astar );
  batteryPercentage = utils.calculateRemainingBatteryPercentage(tabuList, &map, &astar);

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
            + to_string(batteryPercentage) + ", " + to_string(belief_accuracy) + "\n";
  utils.filePutContents(out_log, content, true );

  double totalTimeMCDM = chrono::duration<double,milli> ( endMCDM -startMCDM ).count();
  cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, " << totalTimeMCDM/1000 <<" s, " <<
          totalTimeMCDM/60000 << " m "<< endl;

  utils.printResult(newSensedCells, totalFreeCells, precision, 
                    numConfiguration, travelledDistance, numOfTurning,
                    totalAngle, totalScanTime, accumulated_received_power, &batteryPercentage);

}


Pose getRandomFrontier(list<Pose> nearCandidates){
  long tot_candidates = nearCandidates.size();
  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(0, tot_candidates-1); // guaranteed unbiased
  std::vector<Pose> v{ std::make_move_iterator(std::begin(nearCandidates)),
                    std::make_move_iterator(std::end(nearCandidates)) };
  auto random_integer = uni(rng);
  return v.at(random_integer);
}
