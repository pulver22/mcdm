#include "utils.h"
#include <boost/filesystem.hpp>
#include <fstream>

Utilities::Utilities(){}

Utilities::~Utilities(){}

bool Utilities::contains ( std::list<Pose>& list, Pose& p )
{
  bool result = false;

  std::list<Pose>::iterator findIter = std::find ( list.begin(), list.end(), p );
  if ( findIter != list.end() )
  {
    result = true;
  }

  return result;
}

void Utilities::cleanPossibleDestination2(std::list<Pose> *possibleDestinations, Pose &p) {
  std::list<Pose>::iterator findIter = std::find(possibleDestinations->begin(), possibleDestinations->end(), p);
  if (findIter != possibleDestinations->end()) {
    possibleDestinations->erase(findIter);
  }
}


void Utilities::pushInitialPositions ( dummy::Map map, int x, int y, int orientation, int range, int FOV, double threshold, string actualPose, vector< pair< string, list< Pose > > >* graph2, MCDMFunction *function )
{
  NewRay ray;
  ray.findCandidatePositions ( map,x,y,orientation ,FOV,range );
  vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
  ray.emptyCandidatePositions();
  list<Pose> frontiers;
  vector<pair<long,long> >::iterator it =candidatePosition.begin();
  for ( it; it != candidatePosition.end(); it++ )
  {
    Pose p1 = Pose ( ( *it ).first, ( *it ).second,0 ,range,FOV );
    Pose p2 = Pose ( ( *it ).first, ( *it ).second,180,range,FOV );
    Pose p3 = Pose ( ( *it ).first, ( *it ).second,90,range,FOV );
    Pose p4 = Pose ( ( *it ).first, ( *it ).second,270,range,FOV );
    frontiers.push_back ( p1 );
    frontiers.push_back ( p2 );
    frontiers.push_back ( p3 );
    frontiers.push_back ( p4 );
  }
  EvaluationRecords *record = function->evaluateFrontiers ( frontiers,map,threshold );
  list<Pose>nearCandidates = record->getFrontiers();
  std::pair<string,list<Pose>> pair = make_pair ( actualPose,nearCandidates );
  graph2->push_back ( pair );
}

double Utilities::calculateScanTime ( double scanAngle )
{
  return ( -7.2847174296449998e-006*scanAngle*scanAngle*scanAngle + 2.2131847908245512e-003*scanAngle*scanAngle + 1.5987873410233613e-001*scanAngle + 10 );
}

Pose Utilities::createFromInitialPose ( int x, int y, int orientation, int variation, int range, int FOV )
{
  Pose tmp = Pose ( x,y, ( orientation + variation ) %360,FOV,range );
  return tmp;
}


void Utilities::calculateDistance(list<Pose> history, dummy::Map& map, Astar* astar)
{
    std::list<Pose>::iterator it = history.begin();
    double travelledDistance = 0;
    int numOfTurning = 0;
    // Calculate the overall path connecting these cells
    for ( it; it != prev ( history.end(),1 ); it++ )
    {
        std::list<Pose>::iterator it2 = next ( it,1 );
        string path = astar->pathFind ( ( *it2 ).getX(), ( *it2 ).getY(), ( *it ).getX(), ( *it ).getY(), map );
        travelledDistance = travelledDistance + astar->lengthPath ( path );
        numOfTurning = numOfTurning + astar->getNumberOfTurning ( path );
    }
    cout << "Number of cells: " << history.size() << endl;
    cout << "Num of Turning: " << numOfTurning << endl;
    cout << "Travelled distance (cells): " << travelledDistance << endl;
    cout << "Travelled distance (meters): " << travelledDistance / 2.0 << endl; // Valid only if imgresolution == 1.0 (cell side is 0.5m)
}

void Utilities::updatePathMetrics(int* count, Pose* target, Pose* previous, string actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
    dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int encodedKeyValue, Astar* astar , long* numConfiguration,
    double* totalAngle, double* travelledDistance, int* numOfTurning , double scanAngle)
{
  // Add it to the list of visited cells as first-view
  history->push_back ( function->getEncodedKey ( *target, encodedKeyValue ) );
  // Add it to the list of visited cells from which acting
  tabuList->push_back ( *target );
  // Remove it from the list of candidate position
  cleanPossibleDestination2 ( nearCandidates, *target );
  // Push in the graph the previous robot pose and the new list of candidate position, without the current pose of the robot
  // We don't want to visit this cell again
  std::pair<string,list<Pose>> pair = make_pair ( actualPose, *nearCandidates );
  graph2->push_back ( pair );
  // Calculate the path from the previous robot pose to the current one
  string path = astar->pathFind ( target->getX(), target->getY(), previous->getX(), previous->getY(), *map );
  // Update the distance counting
  *travelledDistance = *travelledDistance + astar->lengthPath(path);
  // Update the turning counting
  *numOfTurning = *numOfTurning + astar->getNumberOfTurning(path);
  // Update the scanning angle
  *totalAngle += scanAngle;
  // Update the number of configurations of the robot along the task
  (*numConfiguration)++;
  // Update counter of iterations
  (*count)++;
}

list<Pose> Utilities::cleanHistory(vector<string>* history, EvaluationRecords* record){
  vector<string>::iterator it_history = history->begin();
  list<Pose> tmp_history;
  for ( it_history; it_history!=prev(history->end(),1); it_history++)
  {
    if ((*it_history).back() == '1')
    {
      tmp_history.push_back(record->getPoseFromEncoding(*it_history));
    }
  }
  return tmp_history;
}

void Utilities::printResult(long newSensedCells, long totalFreeCells, double precision,
                 long numConfiguration, double travelledDistance,
                 int numOfTurning, double totalAngle, double totalScanTime)
{
  cout << "-----------------------------------------------------------------"<<endl;
  cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
  cout << "Total cell visited :" << numConfiguration <<endl;
  cout << "Total travelled distance (meters): " << travelledDistance << endl;
  cout << "Total travel time: " << travelledDistance / 0.5 << "s, " << ( travelledDistance/0.5) /60 << " m"<< endl;
  cout << "I came back to the original position since i don't have any other candidate position"<< endl;
  cout << "Total exploration time (s): " << travelledDistance / 0.5 << endl;
  cout << "Total number of turning: " << numOfTurning << endl;
  cout << "Sum of scan angles (radians): " << totalAngle << endl;
  cout << "Total time for scanning: " << totalScanTime << endl;
  cout << "Total time for exploration: " << travelledDistance/0.5 + totalScanTime << "s, " <<
                                              ( travelledDistance/0.5 + totalScanTime ) /60 << " m" << endl;
  if (newSensedCells < precision * totalFreeCells)
  {
    cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
  } else
  {
    cout << "FINAL: MAP EXPLORED!" << endl;
  }

  cout << "-----------------------------------------------------------------"<<endl;
}

// Usage example: filePutContents("./yourfile.txt", "content", true);
void Utilities::filePutContents(const std::string& name, const std::string& content, bool append ) {
  std::ofstream outfile;
  std::ifstream pFile(name);
  if (outfile.fail()){
    cout << "Error while opening the stream." << endl;
    // cout << "File does not exist! Create a new one!" << endl;
    // outfile.open(name);
    // outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,coverage,numConfiguration,travelledDistance,totalScanTime";
  }
  else
  {
    if (pFile.peek() == std::ifstream::traits_type::eof()){ // file is empty
      cout << "File does not exist! Create a new one!" << endl;
      outfile.open(name);
      outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,coverage,numConfiguration,travelledDistance,totalScanTime" << endl;
    }else{
      cout << "File exists! Appending data!" << endl;
      outfile.open(name, std::ios_base::app);
    }
  }
  outfile << content;
}

// Usage example: filePutContents("./yourfile.txt", "content", true);
void Utilities::saveCoverage(const std::string& name, const std::string& content, bool append ) {
  std::ofstream outfile;
  std::ifstream pFile(name);
  if (outfile.fail()){
    cout << "Error while opening the stream." << endl;
    // cout << "File does not exist! Create a new one!" << endl;
    // outfile.open(name);
    // outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,coverage,numConfiguration,travelledDistance,totalScanTime";
  }
  else
  {
    if (pFile.peek() == std::ifstream::traits_type::eof()){ // file is empty
      cout << "File does not exist! Create a new one!" << endl;
      outfile.open(name);
      outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,numConfiguration,incrementalCoverage" << endl;
    }else{
      cout << "File exists! Appending data!" << endl;
      outfile.open(name, std::ios_base::app);
    }
  }
  outfile << content;
}
