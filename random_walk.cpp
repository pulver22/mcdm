#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>
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
int main ( int argc, char **argv )
{
  auto startMCDM = chrono::high_resolution_clock::now();
  ifstream infile;
  infile.open ( argv[1] );  // the path to the map
  double resolution = atof ( argv[2] );  // the resolution of the map
  double imgresolution = atof ( argv[10] );  // the resolution to use for the planningGrid and RFIDGrid
  dummy::Map map = dummy::Map ( infile,resolution, imgresolution );
  RFIDGridmap myGrid(argv[1], resolution, imgresolution, false);
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
  std::pair<long, long> nextRandomPosition;
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<> distr(0, 359); // define the range
  long x, y;
  int orientation, range;
  double FOV, scanAngle, rxPower, phase;
  string actualPose, encoding;

  do
  {
      content = to_string(w_info_gain) 
                + "," + to_string(w_travel_distance)
                + "," + to_string(w_sensing_time) 
                + "," + to_string(w_rfid_gain)
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
      travelledDistance = travelledDistance + astar.lengthPath ( path );
      previous = target;
      encoding = to_string ( target.getX() ) + to_string ( target.getY() );
      visitedCell.emplace ( encoding,0 );
      // Get the sensing time required for scanning
      target.setScanAngles ( ray.getSensingTime ( &map,x,y,orientation,FOV,range ) );
      // Perform a scanning operation
      double major_axis, minor_axis;
      double focal_length = (range - 1.0) / 2.0; // (X_max - X_min)/2
      major_axis = focal_length + 1.0;  // (focal_length + X_min)
      minor_axis = sqrt(pow(major_axis, 2) - pow(focal_length, 2));
      // cout << "Ellipse axis: " << major_axis << ", " << minor_axis << endl;
      newSensedCells = sensedCells + ray.performSensingOperationEllipse ( &map,x,y,orientation,             target.getScanAngles().first, target.getScanAngles().second, major_axis, minor_axis);
      // Calculate the scanning angle
      scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      // Update the overall scanning time
      totalScanTime += utils.calculateScanTime ( scanAngle*180/PI );
      // Calculare the relative RFID tag position to the robot position
      relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
      // Calculate the received power and phase
      rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
      phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
      // Update the path planning and RFID map
      map.updatePathPlanningGrid ( x, y, range, rxPower - SENSITIVITY);
      myGrid.addEllipse(rxPower - SENSITIVITY, map.getNumGridCols() - target.getX(),  target.getY(), target.getOrientation(), -1.0, range);
      nextRandomPosition = map.getRandomFreeCell();
      // cout << "Next position: " << nextRandomPosition.first << ", " << nextRandomPosition.second << endl;
      orientation = distr(eng);  // get a random orientation between 0 and 359
      orientation = orientation * M_PI / 180.0;
      target.updateFromData(nextRandomPosition, orientation, range, FOV);
      // cout << "Target: " << target.getX() << ", " << target.getY() << endl;
      sensedCells = newSensedCells;
      numConfiguration++;
      // Add target to history and tabulist
      history.push_back(function.getEncodedKey(target, 1));
      tabuList.push_back(target);

  }
  // Perform exploration until a certain coverage is achieved
  while ( sensedCells < precision * totalFreeCells );
  // Plotting utilities
  map.drawVisitedCells ();
  map.printVisitedCells ( history );
  map.drawRFIDScan();
  map.drawRFIDGridScan(myGrid);
  myGrid.saveAs(("/home/pulver/Desktop/MCDM/rfid_result_gridmap.pgm"));

  // cout << "Num configuration: " << numConfiguration << endl;
  // cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;
  //
  // cout << "------------------ HISTORY -----------------" << endl;
  // Calculate which cells have been visited only once
  list<Pose> tmp_history = utils.cleanHistory(&history, &record);
  utils.calculateDistance(tmp_history, &map, &astar );

  // cout << "------------------ TABULIST -----------------" << endl;
  utils.calculateDistance(tabuList, &map, &astar );

  // Trasform distance in meters
  if ( imgresolution == 1.0 ) // Corridor map has a resolution of 0.5 meter per cell
  {
    travelledDistance = travelledDistance/2;
  }


  utils.printResult(newSensedCells, totalFreeCells, precision, numConfiguration, travelledDistance, numOfTurning,
      totalAngle, totalScanTime);
  std::pair<int,int> tag;
  std::pair<int, std::pair<int, int>> value_tag;
  int value = 0;
  value_tag = map.findTagfromGridMap(myGrid);
  tag = value_tag.second;
  cout << "[Grid]RFID pose: [" << tag.second << "," << tag.first << "]" << endl;
  double distance_to_tag = sqrt(pow(absTagX - tag.first, 2) + pow(absTagY - tag.second, 2));
  cout << "Distance to tag: " << to_string(distance_to_tag) << " cells" << endl;
  cout << "-----------------------------------------------------------------"<<endl;
  auto endMCDM = chrono::high_resolution_clock::now();
  content = to_string(w_info_gain) + ","  + to_string(w_travel_distance) + "," + to_string(w_sensing_time) + "," + to_string(w_rfid_gain) + ","
             + to_string(float(newSensedCells)/float(totalFreeCells)) + "," + to_string(numConfiguration) + ","
             + to_string(travelledDistance) + "," + to_string(totalScanTime) + "," 
             + to_string(distance_to_tag) +  "\n";
  utils.filePutContents(out_log, content, true );

  double totalTimeMCDM = chrono::duration<double,milli> ( endMCDM -startMCDM ).count();
  //     cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, " << totalTimeMCDM/1000 <<" s, " <<
  // 		totalTimeMCDM/60000 << " m "<< endl;

}
