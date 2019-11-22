#include "utils.h"
#include <boost/filesystem.hpp>
#include "radio_models/propagationModel.cpp"
#include <fstream>

Utilities::Utilities(double w_info_gain, double w_travel_distance, double w_sensing_time, double w_rfid_gain){
  this->w_info_gain       = w_info_gain;
  this->w_travel_distance = w_travel_distance;
  this->w_sensing_time    = w_sensing_time;
  this->w_rfid_gain       = w_rfid_gain;
}

Utilities::Utilities(){};

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

void Utilities::cleanPossibleDestination2(std::list<Pose> *possibleDestinations, Pose &p) 
{
  std::list<Pose>::iterator findIter = std::find(possibleDestinations->begin(), possibleDestinations->end(), p);
  if (findIter != possibleDestinations->end()) {
    possibleDestinations->erase(findIter);
  }
}


void Utilities::pushInitialPositions ( dummy::Map* map, int x, int y, int orientation, int range, int FOV, double threshold, string actualPose, vector< pair< string, list< Pose > > >* graph2, MCDMFunction *function )
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
  EvaluationRecords *record = function->evaluateFrontiers ( frontiers, map, threshold );
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


void Utilities::calculateDistance(list<Pose> history, dummy::Map* map, Astar* astar)
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
    // cout << "Number of cells: " << history.size() << endl;
    // cout << "Num of Turning: " << numOfTurning << endl;
    // cout << "Travelled distance (cells): " << travelledDistance << endl;
    // cout << "Travelled distance (meters): " << travelledDistance / 2.0 << endl; // Valid only if imgresolution == 1.0 (cell side is 0.5m)
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
  string path = astar->pathFind ( target->getX(), target->getY(), previous->getX(), previous->getY(), map );
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
      // cout << "File does not exist! Create a new one!" << endl;
      outfile.open(name);
      outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,norm_w_info_gain,norm_w_travel_distance,norm_w_sensing_time,norm_w_rfid_gain,coverage,numConfiguration,travelledDistance,totalScanTime" << endl;
    }else{
      // cout << "File exists! Appending data!" << endl;
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
      // cout << "File does not exist! Create a new one!" << endl;
      outfile.open(name);
      outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,numConfiguration,incrementalCoverage" << endl;
    }else{
      // cout << "File exists! Appending data!" << endl;
      outfile.open(name, std::ios_base::app);
    }
  }
  outfile << content;
}


bool Utilities::recordContainsCandidates( EvaluationRecords* record,
                              int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                              dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                              double* totalAngle, double* travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold)
{
  // Set the previous pose equal to the current one (represented by target)
  *previous = *target;
  // Select the new robot destination from the list of candidates
  std::pair<Pose,double> result = function->selectNewPose ( record );
  *target = result.first;
  // If the selected destination does not appear among the cells already visited
  if ( ! this->contains ( *tabuList, *target ))
  {
    // act = true;
    // Add it to the list of visited cells as first-view
    *encodedKeyValue = 1;
    this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                              tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, 
                              travelledDistance, numOfTurning, *scanAngle);
  }
  // ...otherwise, if the seleced cell has already been visited
  else
  {
    // If the graph is empty, stop the navigation
    if ( graph2->size() == 0 ) return true;
    // If there still are more candidates to explore from the last pose in the graph
    if ( graph2->at ( graph2->size()-1 ).second.size() != 0 )
    {
      // cout << "[BT1 - Tabulist]There are visible cells but the selected one is already explored!Come back to second best position from the previous position"<< endl;
      // Remove the current position from possible candidates
      this->cleanPossibleDestination2 ( nearCandidates, *target );
      // Get the list of new candidate position with associated evaluation
      record = function->evaluateFrontiers ( *nearCandidates, map, *threshold );
      // If there are candidate positions
      if ( record->size() != 0 )
      {
        // Select the new pose of the robot
        std::pair<Pose,double> result = function->selectNewPose ( record );
        *target = result.first;
        *encodedKeyValue = 1;
        this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                                  tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, 
                                  travelledDistance, numOfTurning, *scanAngle);
        // Set that we are now in backtracking
        *btMode = true;
      }
      // If there are no more candidate position from the last position in the graph
      else
      {
        // if the graph is now empty, stop the navigation
        if ( graph2->size() == 0 ) return true;
        // Otherwise, select as new position the last cell in the graph and then remove it from there
        string targetString = graph2->at ( graph2->size()-1 ).first;
        graph2->pop_back();
        *target = record->getPoseFromEncoding ( targetString );
      }
    }
    // ... if the graph still does not present anymore candidate positions for its last pose
    else
    {
      // Remove the last element (cell and associated candidate from there) from the graph
      graph2->pop_back();
      // Select as new target, the new last element of the graph
      string targetString = graph2->at ( graph2->size()-1 ).first;
      *target = record->getPoseFromEncoding ( targetString );
      // Save it history as cell visited more than once
      history->push_back ( function->getEncodedKey ( *target,2 ) );
      // cout << "[BT2 - Tabulist]There are visible cells but the selected one is already explored!Come back to two position ago"<< endl;
      count = count + 1;
    }

  }
  return false;
}

bool Utilities::recordNOTContainsCandidates(vector<pair<string,list<Pose>>>* graph2, EvaluationRecords* record, Pose* target, Pose* previous, vector<string>* history,
                                  MCDMFunction* function, int* count){
  // If the graph is empty, stop the navigation
  if ( graph2->size() == 0 ) return true;
  // Select as new target the last one in the graph structure
  string targetString = graph2->at ( graph2->size()-1 ).first;
  // Remove it from the graph
  graph2->pop_back();
  *target = record->getPoseFromEncoding ( targetString );
  int encoding = 0;
  // Check if the selected cell in the graph is the previous robot position
  if ( !target->isEqual ( *previous ) )
  {
    // do nothing...
  }
  // If the selected cell is the old robot position
  else
  {
    // If there are no more cells in the graph, just finish the navigation
    if ( graph2->size() == 0 ) return true;
    // Select the last position in the graph
    string targetString = graph2->at ( graph2->size()-1 ).first;
    // and remove it from the graph
    graph2->pop_back();
    *target = record->getPoseFromEncoding ( targetString );
  }
  // Set the previous pose as the current one
  *previous = *target;
  // Add it in history as cell visited more than once
  history->push_back ( function->getEncodedKey ( *target,2 ) );
  count = count + 1;
  return false;
}


bool Utilities::recordContainsCandidatesBT(EvaluationRecords* record,
                              int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                              dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                              double* totalAngle, double * travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold)
{
  // Find the new destination
  std::pair<Pose,double> result = function->selectNewPose ( record );
  *target = result.first;
  // If this cells has not been visited before
  if ( ! this->contains ( *tabuList, *target ) )
  {
    // Add it to the list of visited cells as first-view
    *encodedKeyValue = 1;
    this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle);
    // Leave the backtracking branch
    *btMode = false;
    nearCandidates->clear();
    // cout << "[BT-MODE4] Go back to previous positions in the graph" << endl;
  }
  // ... otherwise, if the cells has already been visisted
  else
  {
    // If there are other candidates
    if ( nearCandidates->size() != 0 )
    {
      // cout << "[BT-MODE1]Already visited, but there are other candidates" << endl;

      // Remove the destination from the candidate list
      this->cleanPossibleDestination2 ( nearCandidates, *target );
      // Get the candidates with their evaluation
      EvaluationRecords *record = function->evaluateFrontiers ( *nearCandidates, map, *threshold );
      // Select the new destination
      std::pair<Pose,double> result = function->selectNewPose ( record );
      *target = result.first;

      // Add it to the list of visited cells as first-view
      *encodedKeyValue = 1;
      this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle);
    }
    // ...otherwise, if there are no more candidates
    else
    {
      // cout << "[BT-MODE2] Go back to previous positions in the graph" << endl;
      // Select as target the last element in the graph
      string targetString = graph2->at ( graph2->size()-1 ).first;
      // And remove from the graph
      graph2->pop_back();
      *target = record->getPoseFromEncoding ( targetString );
      // Add it to the history of cell as already more than once
      *encodedKeyValue = 2;
      this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle);
      // Leave backtracking
      *btMode = false;
      // Clear candidate list
      nearCandidates->clear();
    }
  }
}

bool Utilities::recordNOTContainsCandidatesBT(EvaluationRecords* record,
                              int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                              dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                              double* totalAngle, double * travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold){
  // Select as new pose, the last cell in the graph
  string targetString = graph2->at ( graph2->size()-1 ).first;
  // and the remove it form the graph
  graph2->pop_back();
  *target = record->getPoseFromEncoding ( targetString );

  // Add it in history as cell visited more than once
  *encodedKeyValue = 2;
  this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                    tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle);
  // Leave backtracking
  *btMode = false;
  // cout << "[BT-MODE3] Go back to previous positions in the graph" << endl;
}

void Utilities::createMultiplePosition(list<Pose> *frontiers, vector<pair<long, long>> *candidatePosition, int range, double FOV)
{
  // For every candidate position, create 8 pose with a different orientation each and consider them as frontiers
  vector<pair<long,long> >::iterator it = candidatePosition->begin();
  for ( it; it != candidatePosition->end(); it++ )
  {
    Pose p1 = Pose ( ( *it ).first, ( *it ).second,0 ,range,FOV );
    Pose p2 = Pose ( ( *it ).first, ( *it ).second,45,range,FOV );
    Pose p3 = Pose ( ( *it ).first, ( *it ).second,90,range,FOV );
    Pose p4 = Pose ( ( *it ).first, ( *it ).second,135,range,FOV );
    Pose p5 = Pose ( ( *it ).first, ( *it ).second,180,range,FOV );
    Pose p6 = Pose ( ( *it ).first, ( *it ).second,225,range,FOV );
    Pose p7 = Pose ( ( *it ).first, ( *it ).second,270,range,FOV );
    Pose p8 = Pose ( ( *it ).first, ( *it ).second,315,range,FOV );
    frontiers->push_back ( p1 );
    frontiers->push_back(p2);
    frontiers->push_back ( p3 );
    frontiers->push_back(p4);
    frontiers->push_back ( p5 );
    frontiers->push_back(p6);
    frontiers->push_back ( p7 );
    frontiers->push_back(p8);

  }
}


void Utilities::updateMaps(vector<pair<double,double>> *tags_coord, dummy::Map* map, 
                            Pose* target, double *txtPower, const double *SENSITIVITY, double *freq, 
                            vector<RFIDGridmap> * RFID_maps_list, 
                            long *x, long *y, int range){
  std::pair<int, int> relTagCoord;
  for (int i = 0; i < tags_coord->size(); i++){
    relTagCoord = map->getRelativeTagCoord((*tags_coord)[i].first, (*tags_coord)[i].second, target->getX(), target->getY());
    // Calculate the received power and phase
    double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, *freq, *txtPower);
    double phase = phaseDifference(relTagCoord.first, relTagCoord.second, *freq);
    // Update the path planning and RFID map
    map->updatePathPlanningGrid ( *x, *y, range, rxPower - *SENSITIVITY);
    if (rxPower < *SENSITIVITY){
      rxPower = 0;
    } else rxPower = 1;
    (*RFID_maps_list)[i].addEllipse(rxPower , map->getNumGridRows() - target->getX(),  target->getY(), target->getOrientation(), -1.0, range);
  }
}


bool Utilities::updateNavigationGraph(int *count, MCDMFunction *function, vector<pair<string,list<Pose>>> *graph2, Pose *target , dummy::Map *map, 
                            long *x, long *y, int *orientation, int *range, double *FOV, double *threshold, string *actualPose){
  if ( count == 0 )
    {
      this->invertedInitial = this->createFromInitialPose ( *x, *y, *orientation,180, *range, *FOV );
      this->eastInitial     = this->createFromInitialPose ( *x, *y, *orientation,90, *range, *FOV );
      this->westInitial     = this->createFromInitialPose ( *x, *y, *orientation,270, *range, *FOV );
      // Calculate other three pose given the strating one
      string invertedPose = function->getEncodedKey ( invertedInitial,0 );
      string eastPose     = function->getEncodedKey ( eastInitial,0 );
      string westPose     = function->getEncodedKey ( westInitial,0 );
      list<Pose> empty ;
      std::pair<string,list<Pose>> pair1 = make_pair ( invertedPose,empty );
      std::pair<string,list<Pose>> pair2 = make_pair ( eastPose,empty );
      std::pair<string,list<Pose>> pair3 = make_pair ( westPose,empty );
      // And add them (with empty candidates) to the graph structure
      graph2->push_back ( pair1 );
      graph2->push_back ( pair2 );
      graph2->push_back ( pair3 );
    }

    // If it's not the first step but we are in one of the initial position (we come back here with backtracking)
    if ( count != 0 && ( target->isEqual ( invertedInitial ) || target->isEqual ( eastInitial ) || target->isEqual ( westInitial ) ) )
    {
      // If there are no more destination in the graph, terminates the navigation
      if ( graph2->size() == 0 ) return true;
      graph2->pop_back();
      *actualPose = function->getEncodedKey ( *target,0 );
      // Add to the graph the initial positions and the candidates from there (calculated inside the function)
      this->pushInitialPositions ( map, *x, *y, *orientation, *range, *FOV, *threshold, *actualPose, graph2, function );
    }
  return false;
}

bool Utilities::forwardMotion(Pose *target, Pose *previous, list<Pose> *frontiers, list<Pose> *nearCandidates, vector<pair<long,long> > *candidatePosition, NewRay *ray, dummy::Map *map, 
                              long *x, long *y, int *orientation, double *FOV, int *range, vector<pair<string,list<Pose>>> *graph2,
                              EvaluationRecords *record, MCDMFunction *function, double *threshold, int *count, vector<string> *history, 
                              list<Pose> *tmp_history, list<Pose> *tabuList, Astar *astar, double *imgresolution, double *travelledDistance,
                              long *sensedCells, long *newSensedCells, long *totalFreeCells, double *totalScanTime, string *out_log, 
                              long *numConfiguration, string *actualPose, int* encodedKeyValue, double *totalAngle, int *numOfTurning,
                              double *scanAngle, bool *btMode)
{
  // If there are no new candidate positions from the current pose of the robot
  if ( candidatePosition->size() == 0 )
  {
    // Find candidates
    ray->findCandidatePositions2 ( map, *x, *y, *orientation, *FOV, *range );
    *candidatePosition = ray->getCandidatePositions();
    ray->emptyCandidatePositions();

    // cout << "No other candidate position" << endl;
    // cout << "----- BACKTRACKING -----" << endl;
    // If the graph contains cells that can be explored
    if ( graph2->size() > 1 )
    {
      // Get the last position in the graph and then remove it
      string targetString = graph2->at ( graph2->size()-1 ).first;
      graph2->pop_back();
      *target = record->getPoseFromEncoding ( targetString );
      // Add it to the history as cell visited more than once
      history->push_back ( function->getEncodedKey ( *target,2 ) );
      // cout << "[BT]No significative position reachable. Come back to previous position" << endl;
      *count = *count + 1;
    }
    //...otherwise, if the graph does not contain cells that can be explored
    // The navigation is finished!
    else
    {
      // cout << "Num configuration: " << numConfiguration << endl;
      // cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;
      // cout << "------------------ HISTORY -----------------" << endl;
      // Retrieve the cell visited only the first time
      *tmp_history = this->cleanHistory(history, record);
      this->calculateDistance(*tmp_history, map, astar );

      // cout << "------------------ TABULIST -----------------" << endl;
      // Calculate the path connecting the cells in the tabulist, namely the cells that are visited one time and couldn't be visite again
      this->calculateDistance(*tabuList, map, astar );

      // Normalise the travel distance in meter
      // NOTE: assuming that the robot is moving at 0.5m/s and the resolution of the map is 0.5m per cell)
      if ( *imgresolution == 1.0 )
      {
        *travelledDistance = *travelledDistance/2;
      }
      // utils.printResult(newSensedCells, totalFreeCells, precision, numConfiguration, travelledDistance, numOfTurning,
      //     totalAngle, totalScanTime);
      string content = to_string(this->w_info_gain) + ","  + to_string(this->w_travel_distance) + "," + to_string(this->w_sensing_time) 
                + "," + to_string(this->w_rfid_gain) + "," + to_string(float(*newSensedCells)/float(*totalFreeCells)) 
                + "," + to_string(*numConfiguration) + "," + to_string(*travelledDistance) + "," + to_string(*totalScanTime) + "\n";
      this->filePutContents(*out_log, content, true );
      exit ( 0 );
    }

    *sensedCells = *newSensedCells;

  }
  //... otherwise, if there are further candidate new position from the current pose of the robot
  else
  {
    // For each candiate position, create many more with different robot orientation
    this->createMultiplePosition(frontiers, candidatePosition, *range, *FOV);
    
    // Evaluate the frontiers and return a list of <frontier, evaluation> pairs
    record = function->evaluateFrontiers ( *frontiers, map, *threshold );
    *nearCandidates = record->getFrontiers();

    // If there are candidate positions
    if ( record->size() != 0 )
    {
      bool break_loop = this->recordContainsCandidates( record, count, target, previous, 
                                actualPose, nearCandidates, graph2, map, function, tabuList, 
                                history, encodedKeyValue, astar , numConfiguration, totalAngle,
                                travelledDistance, numOfTurning , scanAngle, btMode, threshold);
      // cout << "Break: " << break_loop << endl;
      if (break_loop == true) return true;
    }
    // ... otherwise, if there are no candidate positions
    else
    {
      bool break_loop = this->recordNOTContainsCandidates(graph2, record, target,
                                previous, history, function, count);
      if (break_loop == true) return true;
    }

    //NOTE: not requested for testing purpose
    //usleep(microseconds);
    *sensedCells = *newSensedCells;
    frontiers->clear();
    candidatePosition->clear();
    delete record;
  }
  return false;
}

void Utilities::findTags(vector<RFIDGridmap> *RFID_maps_list, vector<pair<double, double>> *tags_coord, dummy::Map *map,
                          string detection_log, string accuracy_log, 
                          int initRange, long numConfiguration)
{
  // Find the tag
  std::pair<int, std::pair<int, int>> value_tag;
  int value = 0;
  std::pair<int, int> tag;
  // FIXME: not accurate, it must not be used
    // tag= map.findTag();
    // cout << "RFID pose: [" << tag.second << "," << tag.first << "]" << endl;
  std::string tags_distance_from_gt;
  double distance_to_tag = 0;
  double accuracy = 0;
  for (int i=0; i < (*RFID_maps_list).size(); i++){
    value_tag = map->findTagfromGridMap((*RFID_maps_list)[i]);
    value = value_tag.first;
    tag = value_tag.second;
    // cout << "[Grid]RFID pose: [" << tag.second << "," << tag.first << "]" << "  ->  GT:[" << tags_coord[i].second << "," << tags_coord[i].first << "]" << endl;
    distance_to_tag = sqrt(pow((*tags_coord)[i].first - tag.first, 2) + pow((*tags_coord)[i].second - tag.second, 2));
    // cout << "Value: " << value << endl;
    // if (value >=2) accuracy = accuracy + 1;
    // cout << "Distance to tag: " << to_string(distance_to_tag) << " cells" << endl;
    if (distance_to_tag <= 15.0) accuracy = accuracy + 1;
    // if (distance_to_tag <= 8.0) {
    //   distance_to_tag = 1;
    // }else distance_to_tag = 0.0;
    tags_distance_from_gt += to_string(distance_to_tag) + ",";
  }
  cout << "Accuracy: " << to_string(accuracy/10.0) << endl;
  tags_distance_from_gt += "\n";
  this->filePutContents(detection_log, tags_distance_from_gt, true);
  accuracy = accuracy / 10.0;
  std::string accuracy_content = to_string(initRange) + "," + to_string(numConfiguration) + "," + to_string(accuracy) + "\n";
  this->filePutContents(accuracy_log, accuracy_content, true);
}

void Utilities::saveRFIDMaps(vector<RFIDGridmap> *RFID_maps_list, string root)
{
  for (int i=0; i < (*RFID_maps_list).size(); i++){
    std::string path = root + to_string(i+1) + ".pgm";
    (*RFID_maps_list)[i].saveAs(path);
  }
}

void Utilities::getEllipseSize(int X_max, int X_min, double *major_axis, double *minor_axis)
{
  double focal_length = (X_max - X_min) / 2.0; // (X_max - X_min)/2
  *major_axis = focal_length + X_min;  // (focal_length + X_min)
  *minor_axis = sqrt(pow(*major_axis, 2) - pow(focal_length, 2));
}