#include "utils.h"
#include <cstring>

Utilities::Utilities(double w_info_gain, double w_travel_distance, double w_sensing_time, double w_rfid_gain, double w_battery_status){
  this->w_info_gain       = w_info_gain;
  this->w_travel_distance = w_travel_distance;
  this->w_sensing_time    = w_sensing_time;
  this->w_rfid_gain       = w_rfid_gain;
  this->w_battery_status  = w_battery_status;
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
  if (possibleDestinations->size() > 0){
    std::list<Pose>::iterator findIter = std::find(possibleDestinations->begin(), possibleDestinations->end(), p);
    if (findIter != possibleDestinations->end()) {
      possibleDestinations->erase(findIter);
    }
  }
  
}


void Utilities::pushInitialPositions ( dummy::Map* map, int x, int y, int orientation, int range, int FOV, double threshold, string actualPose, vector< pair< string, list< Pose > > >* graph2, MCDMFunction *function, RFID_tools *rfid_tools , double *batteryTime)
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
  EvaluationRecords *record = function->evaluateFrontiers ( frontiers, map, threshold, rfid_tools, batteryTime );
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


double Utilities::calculateDistance(list<Pose> history, dummy::Map* map, Astar* astar)
{
    // std::list<Pose>::iterator it = history.begin();
    double travelledDistance = 0;
    int numOfTurning = 0;
    // std::cout << "len: " << history.size() << endl;
    // Calculate the overall path connecting these cells
    for (auto it = history.begin(); it != prev ( history.end(),1 ); it++ )
    {
        std::list<Pose>::iterator it2 = next ( it,1 );
        // std::cout << it->getX() << " " << it->getY() << " : " << it2->getX() << " " << it2->getY() << endl;
        // std::cout << "1" << endl;
        string path = astar->pathFind ( ( *it2 ).getX(), ( *it2 ).getY(), ( *it ).getX(), ( *it ).getY(), map );
        // std::cout << "2" << endl;
        travelledDistance +=  astar->lengthPath ( path );
        // std::cout << "3" << endl;
        numOfTurning += astar->getNumberOfTurning ( path );
        // std::cout << "4" << endl;
    }
    // std::cout << "Number of cells: " << history.size() << endl;
    // std::cout << "Num of Turning: " << numOfTurning << endl;
    // std::cout << "Travelled distance (cells): " << travelledDistance << endl;
  return travelledDistance;
}


double Utilities::calculateRemainingBatteryPercentage(list<Pose> history, dummy::Map* map, Astar* astar)
{
    // std::list<Pose>::iterator it = history.begin();
    double distance, tmp_numOfTurning, translTime, rotTime = 0;
    double batteryTime = MAX_BATTERY;
    // std::cout << "len: " << history.size() << endl;
    // Calculate the overall path connecting these cells
    for (auto it = history.begin(); it != prev ( history.end(),1 ); it++ )
    {
        std::list<Pose>::iterator it2 = next ( it,1 );
        // std::cout << it->getX() << " " << it->getY() << " : " << it2->getX() << " " << it2->getY() << endl;
        // std::cout << "1" << endl;
        string path = astar->pathFind ( ( *it2 ).getX(), ( *it2 ).getY(), ( *it ).getX(), ( *it ).getY(), map );
        // // Update the distance counting
        distance = astar->lengthPath(path);
        tmp_numOfTurning = astar->getNumberOfTurning ( path );
        translTime = distance / TRANSL_SPEED;
        rotTime = tmp_numOfTurning / ROT_SPEED;
        // NOTE: in backtracking we are not phisically moving
        batteryTime -= (translTime + rotTime);  
            // std::cout << "4" << endl;
    }
    batteryTime = 100*batteryTime/MAX_BATTERY;
    // std::cout << "Remaining battery: " << to_string(batteryTime) << endl;
  return batteryTime;
}

void Utilities::updatePathMetrics(int* count, Pose* target, Pose* previous, string actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
    dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int encodedKeyValue, Astar* astar , long* numConfiguration,
    double* totalAngle, double* travelledDistance, int* numOfTurning , double scanAngle, double *batteryTime)
{
  // Add it to the list of visited cells as first-view
  // history->push_back ( function->getEncodedKey ( *target, encodedKeyValue ) );
  for (auto it = tabuList->begin(); it != tabuList->end(); it++){
    this->cleanPossibleDestination2 ( nearCandidates, *it);
  }
  // Remove it from the list of candidate position
  cleanPossibleDestination2 ( nearCandidates, *target );
  // Push in the graph the previous robot pose and the new list of candidate position, without the current pose of the robot
  // We don't want to visit this cell again
  std::pair<string,list<Pose>> pair = make_pair ( actualPose, *nearCandidates );
  // NOTE: if we are in backtracking there is no reason to add a cell already present (it will only make increase the graph)
  if (encodedKeyValue != 2) {
    // Add it to the list of visited cells from which acting
    history->push_back ( function->getEncodedKey ( *target, encodedKeyValue ) );
    tabuList->push_back ( *target );
    graph2->push_back ( pair );
    // Calculate the path from the previous robot pose to the current one
    string path = astar->pathFind ( target->getX(), target->getY(), previous->getX(), previous->getY(), map );
    // // Update the distance counting
    double distance = astar->lengthPath(path);
    double tmp_numOfTurning = astar->getNumberOfTurning ( path );
    *travelledDistance += distance;
    // // Update the turning counting
    *numOfTurning += tmp_numOfTurning;
    double translTime = distance / TRANSL_SPEED;
    double rotTime = tmp_numOfTurning / ROT_SPEED;
    // NOTE: in backtracking we are not phisically moving
    *batteryTime -= (translTime + rotTime);  
    // // Update the scanning angle
    *totalAngle += scanAngle;
    // Update the number of configurations of the robot along the task
    (*numConfiguration)++;
    // Update counter of iterations
    (*count)++;
  } 
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
                 long numConfiguration, double travelledDistance, int numOfTurning, 
                 double totalAngle, double totalScanTime, double accumulated_received_power, double *batteryTime)
{
  std::cout << "-----------------------------------------------------------------"<<endl;
  std::cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells << " ["<< 100*(float)newSensedCells/(float)totalFreeCells << "%]"<< endl;
  std::cout << "Total cell visited :" << numConfiguration <<endl;
  std::cout << "Total travelled distance (cells): " << travelledDistance << endl;
  // std::cout << "Total travel time: " << travelledDistance / 0.5 << "s, " << ( travelledDistance/0.5) /60 << " m"<< endl;
  // std::cout << "I came back to the original position since i don't have any other candidate position"<< endl;
  // std::cout << "Total exploration time (s): " << travelledDistance / 0.5 << endl;
  std::cout << "Total number of turning: " << numOfTurning << endl;
  // std::cout << "Sum of scan angles (radians): " << totalAngle << endl;
  // std::cout << "Total time for scanning: " << totalScanTime << endl;
  // std::cout << "Total time for exploration: " << travelledDistance/0.5 + totalScanTime << "s, " <<
  //                                             ( travelledDistance/0.5 + totalScanTime ) /60 << " m" << endl;
  std::cout << "Accumulated Rx power: " << accumulated_received_power << endl;
  std::cout << "Final battery level: " << *batteryTime << "%" << endl;
  if (newSensedCells < precision * totalFreeCells)
  {
    std::cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
  } else
  {
    std::cout << "FINAL: MAP EXPLORED!" << endl;
  }

  std::cout << "-----------------------------------------------------------------"<<endl;
}

// Usage example: filePutContents("./yourfile.txt", "content", true);
void Utilities::filePutContents(const std::string& name, const std::string& content, bool append ) {
  std::ofstream outfile;
  std::ifstream pFile(name);
  if (outfile.fail()){
    std::cout << "Error while opening the stream." << endl;
    // std::cout << "File does not exist! Create a new one!" << endl;
    // outfile.open(name);
    // outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,coverage,numConfiguration,travelledDistance,totalScanTime";
  }
  else
  {
    if (pFile.peek() == std::ifstream::traits_type::eof()){ // file is empty
      // std::cout << "File does not exist! Create a new one!" << endl;
      outfile.open(name);
      if (name.find("result") != string::npos){
        outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_battery_status,norm_w_info_gain,norm_w_travel_distance,norm_w_sensing_time,norm_w_rfid_gain,norm_w_battery_status,coverage,numConfiguration,travelledDistance,totalScanTime,accumulatedRxPower,batteryStatus,accuracy" << endl;
      }
      else if (name.find("coverage") != string::npos){
        outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_battery_status,norm_w_info_gain,norm_w_travel_distance,norm_w_sensing_time,norm_w_rfid_gain,norm_w_battery_status,numConfiguration,increasingCoverage,travelledDistance" << endl;
      }
      else if (name.find("distance") != string::npos){
        outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_battery_status,tag1,tag2,tag3,tag4,tag5,tag6,tag7,tag8,tag9,tag10" << endl;
      }
      else if (name.find("accuracy") != string::npos){
        outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_battery_status,range,numConfiguration,accuracy" << endl;
      }
    }else{
      // std::cout << "File exists! Appending data!" << endl;
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
    std::cout << "Error while opening the stream." << endl;
    // std::cout << "File does not exist! Create a new one!" << endl;
    // outfile.open(name);
    // outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,coverage,numConfiguration,travelledDistance,totalScanTime";
  }
  else
  {
    if (pFile.peek() == std::ifstream::traits_type::eof()){ // file is empty
      // std::cout << "File does not exist! Create a new one!" << endl;
      outfile.open(name);
      outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_battery_status,numConfiguration,incrementalCoverage" << endl;
    }else{
      // std::cout << "File exists! Appending data!" << endl;
      outfile.open(name, std::ios_base::app);
    }
  }
  outfile << content;
}


bool Utilities::recordContainsCandidates( EvaluationRecords* record,
                              int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                              dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                              double* totalAngle, double* travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold,
                              RFID_tools *rfid_tools, double *batteryTime)
{
  // Set the previous pose equal to the current one (represented by target)
  *previous = *target;
  // Select the new robot destination from the list of candidates
  std::pair<Pose,double> result = function->selectNewPose ( record );
  *target = result.first;
  // If the selected destination does not appear among the cells already visited
  if ( ! this->contains ( *tabuList, *target ))
  {
    // std::cout << "F6-2" << endl;
    // act = true;
    // Add it to the list of visited cells as first-view
    *encodedKeyValue = 1;
    this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle, batteryTime);
  }
  // ...otherwise, if the selected cell has already been visited
  else
  {
    // If the graph is empty, stop the navigation
    if ( graph2->size() == 0 ) return true;
    // If there still are more candidates to explore from the last pose in the graph
    if ( graph2->at ( graph2->size()-1 ).second.size() != 0 )
    { 
      // std::cout <<"F7" << endl;
      // std::cout << "[BT1 - Tabulist]There are visible cells but the selected one is already explored!Come back to second best position from the previous position"<< endl;
      // Remove the current position from possible candidates
      this->cleanPossibleDestination2 ( nearCandidates, *target );
      // Get the list of new candidate position with associated evaluation
      record = function->evaluateFrontiers ( *nearCandidates, map, *threshold, rfid_tools, batteryTime );
      // If there are candidate positions
      if ( record->size() != 0 )
      {
        // std::cout <<"F8" << endl;
        // Select the new pose of the robot
        std::pair<Pose,double> result = function->selectNewPose ( record );
        *target = result.first;
        *encodedKeyValue = 2;
        this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle, batteryTime);
        // Set that we are now in backtracking
        *btMode = true;
      }
      // If there are no more candidate position from the last position in the graph
      else
      {
        // std::cout <<"F9" << endl;
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
      // std::cout <<"F10" << endl;
      // Remove the last element (cell and associated candidate from there) from the graph
      graph2->pop_back();
      // Select as new target, the new last element of the graph
      string targetString = graph2->at ( graph2->size()-1 ).first;
      *target = record->getPoseFromEncoding ( targetString );
      // Save it history as cell visited more than once
      history->push_back ( function->getEncodedKey ( *target,2 ) );
      // std::cout << "[BT2 - Tabulist]There are visible cells but the selected one is already explored!Come back to two position ago"<< endl;
      count++;
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
  count++;
  return false;
}


void Utilities::recordContainsCandidatesBT(EvaluationRecords* record,
                              int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                              dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                              double* totalAngle, double * travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold,
                              RFID_tools *rfid_tools, double *batteryTime)
{
  // Find the new destination
  std::pair<Pose,double> result = function->selectNewPose ( record );
  *target = result.first;
  // If this cells has not been visited before
  if ( ! this->contains ( *tabuList, *target ) )
  { 
    // std::cout << "1" << endl;
    // Add it to the list of visited cells as first-view
    *encodedKeyValue = 1;
    this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle, batteryTime);
    // Leave the backtracking branch
    *btMode = false;
    nearCandidates->clear();
    // std::cout << "[BT-MODE4] Go back to previous positions in the graph" << endl;
  }
  // ... otherwise, if the cells has already been visited
  else
  {
    // If there are other candidates
    if ( nearCandidates->size() != 0 )
    {
      // std::cout << "2" << endl;
      // std::cout << "[BT-MODE1]Already visited, but there are other candidates" << endl;

      // Remove the destination from the candidate list
      for (auto it = tabuList->begin(); it != tabuList->end(); it++){
        this->cleanPossibleDestination2 ( nearCandidates, *it);
      }
      this->cleanPossibleDestination2 ( nearCandidates, *target );
      // Get the candidates with their evaluation
      EvaluationRecords *record = function->evaluateFrontiers ( *nearCandidates, map, *threshold, rfid_tools, batteryTime);
      // Select the new destination
      std::pair<Pose,double> result = function->selectNewPose ( record );
      *target = result.first;
      // if ( ! this->contains ( *tabuList, *target ) ) {
        *btMode = true;
        // *encodedKeyValue = 2;
      // }else{
      //   *btMode = true;
      //   // *encodedKeyValue = 1;
      // }
      // Add it to the list of visited cells as first-view
      *encodedKeyValue = 2;
      this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle, batteryTime);
      

      nearCandidates->clear();
    }
    // ...otherwise, if there are no more candidates
    else
    { 
      // std::cout << "3" << endl;
      // std::cout << "[BT-MODE2] Go back to previous positions in the graph" << endl;
      // Select as target the last element in the graph
      string targetString = graph2->at ( graph2->size()-1 ).first;
      *target = record->getPoseFromEncoding ( targetString );
      *nearCandidates = graph2->at ( graph2->size()-1 ).second;
      // And remove from the graph
      graph2->pop_back();
      // if ( ! this->contains ( *tabuList, *target ) ) {
      //   *btMode = false;
      //   // *encodedKeyValue = 2;
      // }else{
      //   *btMode = true;
      //   // *encodedKeyValue = 1;
      // }
      
      // Add it to the history of cell as already more than once
      *encodedKeyValue = 2;
      this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle, batteryTime);
      // Leave backtracking
      *btMode = true;
      
      // Clear candidate list
      nearCandidates->clear();
      
    }
  }
}

void Utilities::recordNOTContainsCandidatesBT(EvaluationRecords* record,
                              int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                              dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                              double* totalAngle, double * travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold, double* batteryTime){
  // Select as new pose, the last cell in the graph
  graph2->pop_back();
  string targetString = graph2->at ( graph2->size()-1 ).first;
  *nearCandidates = graph2->at ( graph2->size()-1 ).second;
  // and the remove it form the graph
  

  *target = record->getPoseFromEncoding ( targetString );

  // Add it in history as cell visited more than once
  *encodedKeyValue = 2;
  this->updatePathMetrics(count, target, previous, *actualPose, nearCandidates, graph2, map, function,
                      tabuList, history, *encodedKeyValue, astar, numConfiguration, totalAngle, travelledDistance, numOfTurning, *scanAngle, batteryTime);
  // Do not leave backtracking before finding a pose with a record.size() > 0 (new frontiers from there)
  // *btMode = false;
  // std::cout << "[BT-MODE3] Go back to previous positions in the graph" << endl;
}

void Utilities::createMultiplePosition(list<Pose> *frontiers, vector<pair<long, long>> *candidatePosition, int range, double FOV)
{
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


void Utilities::updateMaps( dummy::Map* map, Pose* target,
                            RFID_tools *rfid_tools, bool computeKL=false){
  std::pair<int, int> relTagCoord;
  // std::cout << "----" << std::endl;
  for (int i = 0; i < rfid_tools->tags_coord.size(); i++){
    relTagCoord = map->getRelativeTagCoord((rfid_tools->tags_coord)[i].first, (rfid_tools->tags_coord)[i].second, target->getX(), target->getY());
    // Calculate the received power and phase
    // mfc prev
    //double rxPower = rfid_tools->rm->received_power_friis(relTagCoord.first, relTagCoord.second, *freq, *txtPower);
    double rxPower = rfid_tools->rm->received_power_friis_with_obstacles(target->getX(), target->getY(), target->getOrientation() * 3.141592/180.0, (rfid_tools->tags_coord)[i].first, (rfid_tools->tags_coord)[i].second , 0, rfid_tools->freq);
    //mfc: the above gets the received power between a robot in "target" in METERS and tags_coord[i] in METERS. I'm assuming orientation is in deg.

    double phase = rfid_tools->rm->phaseDifference(relTagCoord.first, relTagCoord.second, rfid_tools->freq);    
    // Update the path planning and RFID map
    map->updatePathPlanningGrid ( target->getX(), target->getY(), target->getRange(), rxPower - rfid_tools->sensitivity);
    //So, robot at pr (x,y,orientation) (long, long, int) receives rxPower,phase,freq from tag i . 
    string tagLayerName = rfid_tools->rm->getTagLayerName(i);
    rfid_tools->rm->addMeasurement(target->getX(), target->getY(), target->getOrientation() , rxPower, phase, rfid_tools->freq, tagLayerName);

    // mfc: dirty trick to plot sequential images of current prob maps
    // static int lineal_index = 0;
    // if ( i == 8 ){
    //   std::cout<<"\t- Tag [" << i << "] at rel position (" << relTagCoord.first << ", " << relTagCoord.second << ") m. " <<std::endl;
    //   std::cout<<"\t- Reading at freq (" << *freq/1e6<< " MHz): (" << (rxPower+30) << ") dBm. ( " << phase << ") rads. " << std::endl;

    //   rfid_tools->rm->saveProbMapDebug("/tmp/",i,lineal_index++,*x,*y, target->getOrientation());
    // }

    // std::cout << "    " << rxPower << std::endl;
    // Moved down after we recast the rxPower
    if (rxPower < rfid_tools->sensitivity){
      rxPower = 0;
    } else rxPower = 1;
    rfid_tools->RFID_maps_list->at(i).addEllipse(rxPower , map->getNumGridRows() - target->getX(),  target->getY(), target->getOrientation(), -1.0, target->getRange());
  }
}

void Utilities::computePosteriorBeliefSingleLayer( dummy::Map* map, Pose* target,
                            RFID_tools *rfid_tools, int tag_id, double len_update){
  std::pair<int, int> relTagCoord;
  // std::cout << "----" << std::endl;  
  relTagCoord = map->getRelativeTagCoord((rfid_tools->tags_coord)[tag_id].first, (rfid_tools->tags_coord)[tag_id].second, target->getX(), target->getY());
  // Calculate the received power and phase
  double rxPower = rfid_tools->rm->received_power_friis_with_obstacles(target->getX(), target->getY(), target->getOrientation() * 3.141592/180.0, (rfid_tools->tags_coord)[tag_id].first, (rfid_tools->tags_coord)[tag_id].second , 0, rfid_tools->freq);
  double phase = rfid_tools->rm->phaseDifference(relTagCoord.first, relTagCoord.second, rfid_tools->freq);    
  //So, robot at pr (x,y,orientation) (long, long, int) receives rxPower,phase,freq from tag i . 
  rfid_tools->rm->addMeasurement(target->getX(), target->getY(), target->getOrientation() , rxPower, phase, rfid_tools->freq, "kl");
  // rfid_tools->rm->addTmpMeasurementRFIDCriterion(target->getX(), target->getY(), target->getOrientation() , rxPower, phase, rfid_tools->freq, tag_id, len_update);
}


bool Utilities::updateNavigationGraph(int *count, MCDMFunction *function, vector<pair<string,list<Pose>>> *graph2, Pose *target , dummy::Map *map, 
                            long *x, long *y, int *orientation, int *range, double *FOV, double *threshold, string *actualPose,
                            RFID_tools *rfid_tools, double *batteryTime){
  if ( count == 0 )
    {
      this->invertedInitial = this->createFromInitialPose ( *x, *y, *orientation,180, *range, *FOV );
      this->eastInitial     = this->createFromInitialPose ( *x, *y, *orientation,90, *range, *FOV );
      this->westInitial     = this->createFromInitialPose ( *x, *y, *orientation,270, *range, *FOV );
      // Calculate other three pose given the starting one
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
      this->pushInitialPositions ( map, *x, *y, *orientation, *range, *FOV, *threshold, *actualPose, graph2, function, rfid_tools, batteryTime );
    }
  return false;
}

bool Utilities::forwardMotion(Pose *target, Pose *previous, list<Pose> *frontiers, list<Pose> *nearCandidates, 
                              vector<pair<long,long> > *candidatePosition, NewRay *ray, dummy::Map *map, 
                              long *x, long *y, int *orientation, double *FOV, int *range, vector<pair<string,list<Pose>>> *graph2,
                              EvaluationRecords *record, MCDMFunction *function, double *threshold, int *count, vector<string> *history, 
                              list<Pose> *tmp_history, list<Pose> *tabuList, Astar *astar, double *imgresolution, double *travelledDistance,
                              long *sensedCells, long *newSensedCells, long *totalFreeCells, double *totalScanTime, string *out_log, 
                              long *numConfiguration, string *actualPose, int* encodedKeyValue, double *totalAngle, int *numOfTurning,
                              double *scanAngle, bool *btMode, RFID_tools *rfid_tools, double *accumulated_received_power, double *precision, double *batteryTime)
{
  // If there are no new candidate positions from the current pose of the robot
  if ( candidatePosition->size() == 0 )
  {
    // std::cout <<"F1" << endl;
    // Find candidates
    ray->findCandidatePositions2 ( map, *x, *y, *orientation, *FOV, *range );
    *candidatePosition = ray->getCandidatePositions();
    ray->emptyCandidatePositions();

    // std::cout << "No other candidate position" << endl;
    // std::cout << "----- BACKTRACKING -----" << endl;
    // If the graph contains cells that can be explored
    if ( graph2->size() > 1 )
    {
      // std::cout <<"F3" << endl;
      // Get the last position in the graph and then remove it
      graph2->pop_back();
      string targetString = graph2->at ( graph2->size()-1 ).first;
      *target = record->getPoseFromEncoding ( targetString );
      *nearCandidates = graph2->at ( graph2->size()-1 ).second;
      // Add it to the history as cell visited more than once
      history->push_back ( function->getEncodedKey ( *target,2 ) );
      // std::cout << "[BT]No significative position reachable. Come back to previous position" << endl;
      *count++;
      *btMode = true;
    }
    //...otherwise, if the graph does not contain cells that can be explored
    // The navigation is finished!
    else
    {
      // std::cout <<"F4" << endl;
      // std::cout << "Num configuration: " << numConfiguration << endl;
      // std::cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;
      // std::cout << "------------------ HISTORY -----------------" << endl;
      // Retrieve the cell visited only the first time
      // *tmp_history = this->cleanHistory(history, record);
      // this->calculateDistance(*tmp_history, map, astar );

      // std::cout << "------------------ TABULIST -----------------" << endl;
      // Calculate the path connecting the cells in the tabulist, namely the cells that are visited one time and couldn't be visite again

      //NOTE Get correct values from tabuList
      *travelledDistance = this->calculateDistance(*tabuList, map, astar );
      *batteryTime = this->calculateRemainingBatteryPercentage(*tabuList, map, astar );

      // Normalise the travel distance in meter
      // NOTE: assuming that the robot is moving at 0.5m/s and the resolution of the map is 0.5m per cell)
      // if ( *imgresolution == 1.0 )
      // {
      //   *travelledDistance = *travelledDistance/2;
      // }
      this->printResult(*newSensedCells, *totalFreeCells, *precision, 
                        *numConfiguration, *travelledDistance, *numOfTurning,
                        *totalAngle, *totalScanTime, *accumulated_received_power, batteryTime);
      string content = to_string(this->w_info_gain) + ","  + to_string(this->w_travel_distance) + "," + to_string(this->w_sensing_time) 
                + "," + to_string(this->w_rfid_gain) + "," + to_string(this->w_battery_status) + "," + to_string(float(*newSensedCells)/float(*totalFreeCells)) 
                + "," + to_string(*numConfiguration) + "," + to_string(*travelledDistance) + "," + to_string(*totalScanTime) + 
                + "," + to_string(*accumulated_received_power) + "\n";
      this->filePutContents(*out_log, content, true );
      exit ( 0 );
    }

    *sensedCells = *newSensedCells;

  }
  //... otherwise, if there are further candidate new position from the current pose of the robot
  else
  {
    // std::cout <<"F2" << endl;
    // For each candidate position, create many more with different robot orientation
    this->createMultiplePosition(frontiers, candidatePosition, *range, *FOV);
    // Evaluate the frontiers and return a list of <frontier, evaluation> pairs
    record = function->evaluateFrontiers ( *frontiers, map, *threshold, rfid_tools, batteryTime );
    *nearCandidates = record->getFrontiers();

    // If there are candidate positions
    if ( record->size() != 0 )
    {
      // std::cout <<"F5" << endl;
      bool break_loop = this->recordContainsCandidates( record, count, target, previous, 
                                actualPose, nearCandidates, graph2, map, function, tabuList, 
                                history, encodedKeyValue, astar , numConfiguration, totalAngle,
                                travelledDistance, numOfTurning , scanAngle, btMode, threshold, rfid_tools, batteryTime);
      // std::cout << "Break: " << break_loop << endl;
      if (break_loop == true) return true;
    }
    // ... otherwise, if there are no candidate positions
    else
    {
      // std::cout <<"F6" << endl;
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

double Utilities::findTags( vector<RFIDGridmap> *RFID_maps_list, vector<pair<double, double>> *tags_coord, dummy::Map *map,
                          string detection_log, string accuracy_log, 
                          int initRange, long numConfiguration,
                          RFID_tools *rfid_tools)
{
  // Find the tag
  std::pair<int, std::pair<int, int>> value_tag, belief_value_tag;
  int value = 0;
  std::pair<int, int> tag, belief_tag;
  // FIXME: not accurate, it must not be used
    // tag= map.findTag();
    // std::cout << "RFID pose: [" << tag.second << "," << tag.first << "]" << endl;
  std::string tags_distance_from_gt = to_string(this->w_info_gain) + "," 
                                    + to_string(this->w_travel_distance) + "," 
                                    + to_string(this->w_sensing_time) + "," 
                                    + to_string(this->w_rfid_gain) + "," 
                                    + to_string(this->w_battery_status) + ",";
  std::string accuracy_content;
  accuracy_content.assign(tags_distance_from_gt);
  double distance_to_tag, belief_distance_to_tag = 0;
  double accuracy, belief_accuracy = 0;
  for (int i=0; i < (*RFID_maps_list).size(); i++){
    value_tag = map->findTagfromGridMap((*RFID_maps_list)[i]);
    belief_value_tag = rfid_tools->rm->findTagFromBeliefMap(i);
    // value = value_tag.first;
    tag = value_tag.second;
    belief_tag = belief_value_tag.second;
    belief_tag.first = belief_tag.first;
    belief_tag.second = belief_tag.second;
    // std::cout << "[Grid]RFID pose: [" << tag.second << "," << tag.first << "]" << "  ->  GT:[" << tags_coord[i].second << "," << tags_coord[i].first << "]" << endl;
    distance_to_tag = sqrt(pow((*tags_coord)[i].first - tag.first, 2) + pow((*tags_coord)[i].second - tag.second, 2));
    belief_distance_to_tag = sqrt(pow((*tags_coord)[i].first - belief_tag.first, 2) + pow((*tags_coord)[i].second - belief_tag.second, 2));
    // std::cout << "Value: " << value << endl;
    // if (value >=2) accuracy = accuracy + 1;
    // std::cout << "Distance to tag: " << to_string(distance_to_tag) << " cells" << endl;
    std::cout << "------" << "[" << i << "]------" << endl;
    // std::cout << "[GT]     Tag: " << to_string((*tags_coord)[i].first) << ", " << to_string((*tags_coord)[i].second) << endl;
    // std::cout << "[Belief] Tag: " << belief_tag.first << "," << belief_tag.second << endl;
    std::cout << "Belief_Distance to tag: " << to_string(belief_distance_to_tag) << " cells" << endl;
    if (distance_to_tag <= 10.0) accuracy = accuracy + 1;
    if (belief_distance_to_tag <= 10.0)
    {
      belief_accuracy = belief_accuracy + 1;
      // std::cout << "Tag: " << i << " found" << std::endl;
    } 

    // if (distance_to_tag <= 8.0) {
    //   distance_to_tag = 1;
    // }else distance_to_tag = 0.0;
    tags_distance_from_gt += to_string(belief_distance_to_tag) + ",";
  }
  accuracy = accuracy / (*RFID_maps_list).size();
  belief_accuracy = belief_accuracy / (*RFID_maps_list).size();
  std::cout << "Accuracy: " << to_string(accuracy) << endl;
  std::cout << "Belief_Accuracy: " << to_string(belief_accuracy) << endl;
  tags_distance_from_gt += "\n";
  this->filePutContents(detection_log, tags_distance_from_gt, true);
  accuracy_content += to_string(initRange) + "," + to_string(numConfiguration) + "," + to_string(belief_accuracy) + "\n";
  this->filePutContents(accuracy_log, accuracy_content, true);
  return belief_accuracy;
}

void Utilities::saveRFIDMaps(vector<RFIDGridmap> *RFID_maps_list, string root)
{
  for (int i=0; i < (*RFID_maps_list).size(); i++){
    std::string path = root + to_string(i) + ".pgm";
    (*RFID_maps_list)[i].saveAs(path);
  }
}

void Utilities::saveRFIDMapsWithGroundTruths(vector<RFIDGridmap> *RFID_maps_list, vector<pair<double, double>> *tags_coord, string root)
{
  for (int i=0; i < (*RFID_maps_list).size(); i++){
    std::string path = root + to_string(i+1) + ".pgm";
    (*RFID_maps_list)[i].saveAsWithGroundTruth(path, (*tags_coord)[i].first,(*tags_coord)[i].second);
  }
}


void Utilities::getEllipseSize(int X_max, int X_min, double *major_axis, double *minor_axis)
{
  double focal_length = (X_max - X_min) / 2.0; // (X_max - X_min)/2
  *major_axis = focal_length + X_min;  // (focal_length + X_min)
  *minor_axis = sqrt(pow(*major_axis, 2) - pow(focal_length, 2));
}