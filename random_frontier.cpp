#include <algorithm>
#include <iostream>
#include <iterator>
#include "map.h"
#include "newray.h"
#include "mcdmfunction.h"
#include "Criteria/traveldistancecriterion.h"
#include "radio_models/propagationModel.cpp"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>
#include <time.h>
#include <ctime>
#include "utils.h"
// #include "RFIDGridmap.h"



using namespace std;
using namespace dummy;
// bool contains ( std::list< Pose >& list, Pose& p );
// void cleanPossibleDestination2 ( std::list< Pose > &possibleDestinations, Pose& p );
// void pushInitialPositions ( dummy::Map map, int x, int y, int orientation,  int range, int FOV, double threshold,
//                             string actualPose, vector< pair< string, list< Pose > > > *graph2, MCDMFunction *function);
// double calculateScanTime ( double scanAngle );
// void calculateDistance(list<Pose> list, dummy::Map& map, Astar* astar);
// Pose createFromInitialPose ( int x, int y, int orientation, int variation, int range, int FOV );
// void updatePathMetrics(int* count, Pose* target, Pose* previous, string actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
//                        dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int encodedKeyValue, Astar* astar , long* numConfiguration,
//                        double* totalAngle, double * travelledDistance, int* numOfTurning , double scanAngle);
// list<Pose> cleanHistory(vector<string>* history, EvaluationRecords* record_history);
// void printResult(long newSensedCells, long totalFreeCells, double precision, long numConfiguration, double travelledDistance,
//                  int numOfTurning, double totalAngle, double totalScanTime);
// void filePutContents(const std::string& name, const std::string& content, bool append = false);

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
  RFIDGridmap myGrid(argv[1], resolution, imgresolution, false);
  cout << "Map dimension: " << map.getNumGridCols() << " : "<<  map.getNumGridRows() << endl;
  int gridToPathGridScale = map.getGridToPathGridScale();
  // i switched x and y because the map's orientation inside and outside programs are different
  long initX = static_cast<long>( atoi ( argv[4] ) *imgresolution );  // initial X-position of the robot in map frame
  long initY = static_cast<long>( atoi ( argv[3] ) *imgresolution );  // initial Y-position of the robot in map frame
  std::cout << "initX: " << initX << " initY: " << initY << std::endl;
  int initOrientation = atoi ( argv[5] );  // initial orientation of the robot in map frame
  double initFov = atoi ( argv[7] );  // initial FOV of the robot sensor
  initFov = initFov * PI /180;
  int initRange = atoi ( argv[6] );
  double precision = atof ( argv[8] );
  double threshold = atof ( argv[9] );
  // RFID
  double absTagX = std::stod(argv[12]); // m.
  double absTagY = std::stod(argv[11]); // m.
  double freq = std::stod(argv[13]); // Hertzs
  double txtPower = std::stod(argv[14]); // dBs
  std::pair<int, int> relTagCoord;
  // MCDM Matrix weights
  double w_info_gain = atof(argv[15]);
  double w_travel_distance = atof(argv[16]);
  double w_sensing_time = atof(argv[17]);
  double w_rfid_gain = atof(argv[18]);
  std::string out_log (argv[19]);
  std::string coverage_log (argv[20]);
  //x,y,orientation,range,FOV
  Utilities utils;

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

  do
  {
    // If we are doing "forward" navigation towards cells never visited before
    if ( btMode == false )
    {

      content = to_string(w_info_gain) + ","  + to_string(w_travel_distance)
                + "," + to_string(w_sensing_time) + "," + to_string(w_rfid_gain)
                + "," + to_string(numConfiguration) + ","
                + to_string(100 * float(newSensedCells)/float(totalFreeCells)) + "\n";
      utils.filePutContents(coverage_log, content, true );
      long x = target.getX();
      long y = target.getY();
      int orientation = target.getOrientation();
      int range = target.getRange();
      double FOV = target.getFOV();
      string actualPose = function.getEncodedKey ( target,0 );
      map.setCurrentPose ( target );
      string encoding = to_string ( target.getX() ) + to_string ( target.getY() );
      visitedCell.emplace ( encoding,0 );
      // Get the sensing time required for scanning
      target.setScanAngles ( ray.getSensingTime ( &map,x,y,orientation,FOV,range ) );
      // Perform a scanning operation
      newSensedCells = sensedCells + ray.performSensingOperation ( &map,x,y,orientation,FOV,range, target.getScanAngles().first, target.getScanAngles().second );
      // Calculate the scanning angle
      double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      // Update the overall scanning time
      totalScanTime += utils.calculateScanTime ( scanAngle*180/PI );
      // Calculare the relative RFID tag position to the robot position
      relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
      // Calculate the received power and phase
      double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
      double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
      // Update the path planning and RFID map
      map.updatePathPlanningGrid ( x, y, range, rxPower - SENSITIVITY);
      myGrid.addEllipse(rxPower - SENSITIVITY, map.getNumGridCols() - target.getX(),  target.getY(), target.getOrientation(), -1.0, range);
      // Search for new candidate position
      ray.findCandidatePositions ( &map,x,y,orientation,FOV,range );
      vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
      ray.emptyCandidatePositions();
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
        utils.pushInitialPositions ( &map, x, y,orientation, range,FOV, threshold, actualPose, &graph2, &function );
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
              totalAngle, totalScanTime);
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
        EvaluationRecords *record = function.evaluateFrontiers ( frontiers, &map, threshold );
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
                &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
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
              record = function.evaluateFrontiers ( nearCandidates, &map, threshold );
              // If there are candidate positions
              if ( record->size() != 0 )
              {
                // Select the new pose of the robot
                target = getRandomFrontier(nearCandidates);
                // std::pair<Pose,double> result = function.selectNewPose ( record );
                // target = result.first;
                encodedKeyValue = 1;
                utils.updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                                  &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
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
      travelledDistance = travelledDistance + astar.lengthPath ( path );
      // cout << "BT: " << astar.lengthPath ( path ) << endl;
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
      // Calculate the relative coordinate to the robot of the RFID tag
      relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
      // Calculate received power and phase
      double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
      double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
      map.updatePathPlanningGrid ( x, y, range, rxPower - SENSITIVITY );
      myGrid.addEllipse(rxPower - SENSITIVITY, map.getNumGridCols() - target.getX(), target.getY(), target.getOrientation(), -0.5, 7.0);
      // Remove the current pose from the list of possible candidate cells
      utils.cleanPossibleDestination2 ( &nearCandidates,target );
      // Get the list of the candidate cells with their evaluation
      EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates, &map, threshold );

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
                            &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
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
            EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates, &map, threshold );
            // Select the new destination
            // std::pair<Pose,double> result = function.selectNewPose ( record );
            // target = result.first;
            target = getRandomFrontier(nearCandidates);

            // Add it to the list of visited cells as first-view
            encodedKeyValue = 1;
            utils.updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                              &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
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
                              &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
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
                          &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
        // Leave backtracking
        btMode = false;
        // cout << "[BT-MODE3] Go back to previous positions in the graph" << endl;
      }
      delete record;
    }
  }
  // Perform exploration until a certain coverage is achieved
  while ( sensedCells < precision * totalFreeCells );
  // Plotting utilities
  map.drawVisitedCells ();
  map.printVisitedCells ( history );
  map.drawRFIDScan();
  map.drawRFIDGridScan(myGrid);
  myGrid.saveAs(("/home/pulver/Desktop/MCDM/rfid_result_gridmap.pgm"));

  cout << "Num configuration: " << numConfiguration << endl;
  cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;

  cout << "------------------ HISTORY -----------------" << endl;
  // Calculate which cells have been visited only once
  list<Pose> tmp_history = utils.cleanHistory(&history, &record);
  utils.calculateDistance(tmp_history, &map, &astar );

  cout << "------------------ TABULIST -----------------" << endl;
  utils.calculateDistance(tabuList, &map, &astar );

  // Trasform distance in meters
  if ( imgresolution == 1.0 ) // Corridor map has a resolution of 0.5 meter per cell
  {
    travelledDistance = travelledDistance/2;
  }


  utils.printResult(newSensedCells, totalFreeCells, precision, numConfiguration, travelledDistance, numOfTurning,
      totalAngle, totalScanTime);
  content = to_string(w_info_gain) + ","  + to_string(w_travel_distance) + "," + to_string(w_sensing_time) + "," + to_string(w_rfid_gain) + ","
             + to_string(float(newSensedCells)/float(totalFreeCells)) + "," + to_string(numConfiguration) + ","
             + to_string(travelledDistance) + "," + to_string(totalScanTime) + "\n";
  utils.filePutContents(out_log, content, true );
  // Find the tag
  std::pair<int,int> tag = map.findTag();
  cout << "RFID pose: [" << tag.second << "," << tag.first << "]" << endl;
  tag = map.findTagfromGridMap(myGrid);
  cout << "[Grid]RFID pose: [" << tag.second << "," << tag.first << "]" << endl;
  cout << "-----------------------------------------------------------------"<<endl;
  auto endMCDM= chrono::high_resolution_clock::now();

  double totalTimeMCDM = chrono::duration<double,milli> ( endMCDM -startMCDM ).count();
  //     cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, " << totalTimeMCDM/1000 <<" s, " <<
  // 		totalTimeMCDM/60000 << " m "<< endl;

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
