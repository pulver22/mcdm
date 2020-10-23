#ifndef UTILS_H
#define UTILS_H

#include "pose.h"
#include "mcdmfunction.h"
#include "evaluationrecords.h"
#include "PathFinding/astar.h"
#include "newray.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include "constants.h"


using namespace dummy;



class Utilities
{
public:
  Utilities(double w_info_gain, double w_travel_distance, double w_sensing_time, double w_rfid_gain, double w_battery_status);
  Utilities();
  ~Utilities();
  
  /**
   * Check if a Pose object is already present within a list
   * 
   * @param list: the list of Pose objects
   * @param p: the Pose object to look for
   */
  bool contains ( std::list< Pose > *list, Pose *p );
 
  /**
   * Remove a Pose object from a list of Pose objects
   *
   *  @param possibleDestinations: list of Pose objects
   * @param p: the object to removeS
   */
  void cleanPossibleDestination2(std::list<Pose> *possibleDestinations, Pose &p);
  
  /**
   * Calculate the frontiers from the starting position and add it to the graph structure
   * 
   * @param map: a copy of the map
   * @param x: the x-coordinate of the robot expressed in cells
   * @param y: the y-coordinate of the robot expressed in cells
   * @param orientation: the orientation of the robot expressed in degrees
   * @param range: the range of the robot's sensor expressed in cells
   * @param threshold: the threshold value to discard unuseful frontiers
   * @param actualPose: encoding of the current robot pose
   * @param graph2: the structure where to save current pose and its candidate positions
   * @param function: the MCDM function object
   */
  void pushInitialPositions (dummy::Map *map, int x, int y, int orientation,
                             int range, int FOV, double threshold,
                             string actualPose,
                             vector< pair< string, list< Pose > > > *graph2,
                             MCDMFunction *function, RFID_tools *rfid_tools, 
                             double *batteryTime, 
                             bool *explorationCompleted);
  /**
   * Calculate the time required for performing a scan with the TDLAS sensor
   * 
   * @param scanAngle: the angle to scan
   */
  double calculateScanTime ( double scanAngle );
  
  /**
   * Calculate the length of a path
   *
   * @param list: a list of Pose lying on the path
   * @param map: a copy of the map
   * @param astar: the astar object used for parsing the map and calculating the distance
   * @return the distance travelled by the robot along the entire path
   */
  double calculateDistance(list<Pose> list, dummy::Map* map, Astar* astar);

  /**
   * Calculate the length of a path
   *
   * @param list: a list of Pose lying on the path
   * @param map: a copy of the map
   * @param astar: the astar object used for parsing the map and calculating the distance
   * @return the final battery percentage of the robot
   */
  double calculateRemainingBatteryPercentage(list<Pose> list, dummy::Map* map, Astar* astar);
  
  /**
   * Create a new Pose object starting from another one
   * 
   * @param x: the x-coordinate of the robot expressed in cell
   * @param y: the y-coordinate of the robot expressed in cell
   * @param orientation: the orientation of the robot expressed in degrees
   * @param range: the range of the robot's sensor expressed in cells
   * @param variation: an additional angle to sum to the orientation
   * @param FOV: the scanning angle of the sensor
   */
  Pose createFromInitialPose (int x, int y, int orientation, int variation,
                              int range, int FOV );
  /**
   * After moving, update the list of cell visited, those which must not be
   * visited again, the distance travelled, numOfTurning, totalAngle and numConfiguration
   * @param count: the iteration count of the MCDM algorithm
   * @param target: the next position of the robot
   * @param previous: the current position of the robot
   * @param actualPose: the encoding of the current pose of the robot
   * @param nearCandidates: the list of possible destination for the robot
   * @param graph2: the structure containing positions and their nearCandidates
   * @param map: a copy of the map
   * @param function: a MCDM function object
   * @param tabuList: the list of Pose which cannot be assumed again
   * @param history: list of encoding of all visited cells by the robot
   * @param encodedKeyValue: a value representing which kind of encoding we want
   * @param astar: a copy of Astar object for calculating the distance between two pose
   * @param numConfiguration: number of configuration assumed by the robot so far
   * @param totalAngle: scanning angle performed so far
   * @param travelledDistance: length traversed by the robot so far
   * @param numOfTurning: number of rotation of 45 deg performed by the robot so far
   * @param scanAngle: angle scanned in the current iteration
   * @param batteryTime: remaining battery for the robot
  */
  void updatePathMetrics(int* count, Pose* target, Pose* previous, string actualPose,
                         list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                         dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList,
                         vector<string>* history, int encodedKeyValue, Astar* astar ,
                         long* numConfiguration, double* totalAngle,
                         double * travelledDistance, int* numOfTurning , double scanAngle,
                         double* batteryTime);
  /**
   * Get the list of Pose assumed only once along the traversed path
   *
   * @param history: the list of Pose assumed by the robot one or more times (during bakctracking)
   * @param record: evaluationRecords object required for converting a string to a Pose
   */
  list<Pose> cleanHistory(vector<string>* history, EvaluationRecords* record);
  
  /**
   * Print in console a summary of the exploration task
   *
   * @param newSensedCells: number of scanned cells
   * @param totalFreeCells: total number of free cells in the map
   * @param precision: the level of coverage we wanted to satisfy
   * @param numConfiguration: number of times the robot stop for a scanning operation
   * @param travelledDistance: the distance covered by the robot (expressed in cells)
   * @param numOfTurning: number of rotation (45 deg) performed by the robot
   * @param totalAngle: total scanning angle covered by the robot
   * @paam totalScanTime: total scanning time
   */
  void printResult(long newSensedCells, long totalFreeCells, double precision,
                   long numConfiguration, double travelledDistance, int numOfTurning, 
                   double totalAngle, double totalScanTime, double accumulated_received_power, double *batteryTime);
  /**
   * Save on file the exploration results
   * 
   * @param name: the output file name
   * @param content: the sring to append to the file
   * @param append: if the file needs to be created from scratch or we append
   */
  void filePutContents(const std::string& name, const std::string& content,
                       bool append = false);

  /**
   * Save on file the exploration results
   * 
   * @param name: the output file name
   * @param content: the sring to append to the file
   * @param append: if the file needs to be created from scratch or we append
   */
  void saveCoverage(const std::string& name, const std::string& content,
                        bool append = false);
  /**
    * What to do when we are doing forward motion and there are candidates to explore from the considered position
    * 
    * @param record: list of candiate position
    * @param count: the iteration count of the MCDM algorithm
    * @param target: the next position of the robot
    * @param previous: the current position of the robot
    * @param actualPose: the encoding of the current pose of the robot
    * @param nearCandidates: the list of possible destination for the robot
    * @param graph2: the structure containing positions and their nearCandidates
    * @param map: a copy of the map
    * @param function: a MCDM function object
    * @param tabuList: the list of Pose which cannot be assumed again
    * @param history: list of encoding of all visited cells by the robot
    * @param encodedKeyValue: a value representing which kind of encoding we want
    * @param astar: a copy of Astar object for calculating the distance between two pose
    * @param numConfiguration: number of configuration assumed by the robot so far
    * @param totalAngle: scanning angle performed so far
    * @param travelledDistance: length traversed by the robot so far
    * @param numOfTurning: number of rotation of 45 deg performed by the robot so far
    * @param scanAngle: angle scanned in the current iteration
    * @param btMode: if doing forward motion or backtracking
    * @param threshold: to cut frontiers
  */
  bool recordContainsCandidates(EvaluationRecords* record, 
                              int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                              dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                                double* totalAngle, double * travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold,
                                RFID_tools *rfid_tools, double *batteryTime, bool *explorationCompleted);
  
  /**
    * What to do when we are doing forward motion and there are no more candidates to explore from the considered position
    * 
    * @param graph2: the structure containing positions and their nearCandidates
    * @param record: list of candiate position
    * @param target: the next position of the robot
    * @param previous: the current position of the robot
    * @param history: list of encoding of all visited cells by the robot
    * @param function: a MCDM function object
    * @param count: the iteration count of the MCDM algorithm
  */
  bool recordNOTContainsCandidates(vector<pair<string,list<Pose>>>* graph2, EvaluationRecords* record, Pose* target, Pose* previous, vector<string>* history,
                                    MCDMFunction* function, int* count);
  
  /**
    * What to do when we are in backtracking and there are candidates to explore from the considered position
    * 
    * @param record: list of candiate position
    * @param count: the iteration count of the MCDM algorithm
    * @param target: the next position of the robot
    * @param previous: the current position of the robot
    * @param actualPose: the encoding of the current pose of the robot
    * @param nearCandidates: the list of possible destination for the robot
    * @param graph2: the structure containing positions and their nearCandidates
    * @param map: a copy of the map
    * @param function: a MCDM function object
    * @param tabuList: the list of Pose which cannot be assumed again
    * @param history: list of encoding of all visited cells by the robot
    * @param encodedKeyValue: a value representing which kind of encoding we want
    * @param astar: a copy of Astar object for calculating the distance between two pose
    * @param numConfiguration: number of configuration assumed by the robot so far
    * @param totalAngle: scanning angle performed so far
    * @param travelledDistance: length traversed by the robot so far
    * @param numOfTurning: number of rotation of 45 deg performed by the robot so far
    * @param scanAngle: angle scanned in the current iteration
    * @param btMode: if doing forward motion or backtracking
  */
  void recordContainsCandidatesBT(EvaluationRecords* record, 
                                int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                                dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                                double* totalAngle, double * travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold,
                                RFID_tools *rfid_tools, double *batteryTime, bool *explorationCompleted);
  
  /**
   * What to do when we are in backtracking and there are no more candidates to explore from the considered position
   * 
   * @param record: list of candiate position
   * @param count: the iteration count of the MCDM algorithm
   * @param target: the next position of the robot
   * @param previous: the current position of the robot
   * @param actualPose: the encoding of the current pose of the robot
   * @param nearCandidates: the list of possible destination for the robot
   * @param graph2: the structure containing positions and their nearCandidates
   * @param map: a reference to the map
   * @param function: a MCDM function object
   * @param tabuList: the list of Pose which cannot be assumed again
   * @param history: list of encoding of all visited cells by the robot
   * @param encodedKeyValue: a value representing which kind of encoding we want
   * @param astar: a copy of Astar object for calculating the distance between two pose
   * @param numConfiguration: number of configuration assumed by the robot so far
   * @param totalAngle: scanning angle performed so far
   * @param travelledDistance: length traversed by the robot so far
   * @param numOfTurning: number of rotation of 45 deg performed by the robot so far
   * @param scanAngle: angle scanned in the current iteration
   * @param btMode: if doing forward motion or backtracking
   * @param threshold: to cut frontiers
   * @param rm: is the radar_model
   * @param accumulated_received_power: the total received power
   * @param batteryTime: the remaining battery time for the robot
   */
  void recordNOTContainsCandidatesBT(EvaluationRecords* record, 
                                int* count, Pose* target, Pose* previous, string* actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                                dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int* encodedKeyValue, Astar* astar , long* numConfiguration,
                                double* totalAngle, double * travelledDistance, int* numOfTurning , double* scanAngle, bool* btMode, double* threshold, double* batteryTime);

  /**
   * For every candidate position, create 8 pose with a different orientation each and consider them as frontiers
   * 
   * @param frontiers: list of frontiers to evaluate
   * @param candidatePosition: list of cells on the border scanned/unscanned environment
   * @param range: sensor range
   * @param FOV: sensor FOV
   */
  void createMultiplePosition(list<Pose> *frontiers, vector<pair<long, long>> *candidatePosition, int range, double FOV);

  /**
   * Update the navigation map and each tags belief map
   * 
   * @param map: the object map (with all its levels)
   * @param target: the next destination of the robot
   * @param rfid_tools: various utility for RFID related operations
   * @param computeKL: flag to compute posterior distribution for calculating KL-div
   */
  void updateMaps(dummy::Map* map, Pose* target, RFID_tools *rfid_tools, bool computeKL);

  /**
   * Update the navigation map and each tags belief map
   * 
   * @param map: the object map (with all its levels)
   * @param target: the next destination of the robot
   * @param rfid_tools: various utility for RFID related operations
   * @param tag_id: the belief map on which computing the posterior
   * @param len_update: the size of the submap to use for computing the posterior
   */
  void computePosteriorBeliefSingleLayer(dummy::Map* map, Pose* target, RFID_tools *rfid_tools, int tag_id, double len_update);

  /**
   * Update the navigation graph structure with the current position
   * 
   * @param count: algorithm iteration
   * @param function: MCDM function for evaluating frontiers
   * @param graph2: navigation graph structure keeping track of visited cells
   * @param target: destination of the robot
   * @param map: a reference to the map object
   * @param x: the current x-coordinate of the robot
   * @param y: the current y-coordinate of the robot
   * @param orientation: the current orientation of the robot
   * @param range: the sensor range
   * @param FOV: the sensor FOV
   * @param threshold: thereshold for discarting frontiers
   * @param actualPose: encoding of the current pose of the robot
   * @param rfid_tools: various utility for RFID related operations
   * @param batteryTime: how much battery is left to the robot
   * @return true if the navigation is finished (graph empty), false otherwise
   */
  bool updateNavigationGraph(int *count, MCDMFunction *function, vector<pair<string,list<Pose>>> *graph2, Pose *target , dummy::Map *map, 
                            long *x, long *y, int *orientation, int *range, double *FOV, double *threshold, string *actualPose,
                            RFID_tools *rfid_tools, double *batteryTime, bool *explorationCompleted);

  /**
   * Main exploration code to follow when the robot is not performing backtracking.
   * 
   * @param target: the next destination of the robot
   * @param previous: the previous (current) position of the robot
   * @param frontiers: list of existing frontiers
   * @param nearCandidates: list of existing frontiers (can be removed to reuse variable frontiers)
   * @param candidatePosition: list of candidate cells betwen scanned/unscanned area which must be evaluated
   * @param ray: ray object for finding frontiers using raycasting
   * @param map: a reference to the map
   * @param x: current x-coord of the robot
   * @param y: current y-coord of the robot
   * @param orientation: current orientation of the robot
   * @param FOV: sensor FOV
   * @param range: sensor range 
   * @param graph2: navigation graph structure listing the visited cells
   * @param record: object providing utilities for the frontiers
   * @param function: the MCDM function for evaluating frontiers
   * @param threshold: the threshold for discarding not relevant frontiers
   * @param count: the algorithm iteration number
   * @param history: list of cells visited (also more than once)
   * @param sensedCells: number of cells scanned so far
   * @param newSensedCells: number of cells updated to the current iteration
   * @param totalFreeCells: total number of free cells in the map
   * @param totalScanTime: total time required by scanning operation
   * @param out_log: path of the result log
   * @param numConfiguration: number of configuration assumed by the robot
   * @param actualPose: encoding of the current pose of the robot
   * @param encodedKeyValue: defines if the cells is new or already been visited
   * @param totalAngle: total of the scanning angle accumulated during the operation
   * @param numOfTurning: total number of turnings the robot performs while navigating
   * @param scanAngle: current scanning angle of the sensor
   * @param btMode: boolean for forward_motion (false) or backtracking (true)
   * @param rfid_tools: various utility for RFID
   * @param accumulated_received_power: sum over time of the accumulated received power
   * @param precision: percentage of map to cover
   * @param batteryTime: time left to robot battery
   * @return true if the navigation is finished, true otherwise
   */
  bool forwardMotion(Pose *target, Pose *previous,list<Pose> *frontiers, list<Pose> *nearCandidates, vector<pair<long,long> > *candidatePosition, NewRay *ray, dummy::Map *map, 
                      long *x, long *y, int *orientation, double *FOV, int *range, vector<pair<string,list<Pose>>> *graph2,
                      EvaluationRecords *record, MCDMFunction *function, double *threshold, int *count, vector<string> *history, 
                      list<Pose> *tmp_history, list<Pose> *tabuList, Astar *astar, double *imgresolution, double *travelledDistance,
                      long *sensedCells, long *newSensedCells, long *totalFreeCells, double *totalScanTime, string *out_log,
                      long *numConfiguration, string *actualPose, int* encodedKeyValue, double *totalAngle, int *numOfTurning,
                      double *scanAngle, bool *btMode, RFID_tools *rfid_tools, double *accumulated_received_power, double *precision, double *batteryTime, bool *explorationCompleted);

  /**
   * At the end of the exploration, look for the region with highest intensity in
   * the belief map and compute the detection accuracy using the ground truth position.
   *
   * @param RFID_maps_list: the list of belief maps
   * @param tags_coord: the coordinates of all the tags
   * @param map: a reference to the map
   * @param detection_log: the path where to save detection results (distance from tags)
   * @param accuracy_log: the path where to save detection accuracy results (number of tags correctly found)
   * @param initRange: sensor range
   * @param numConfiguration: number of configuration of the robot
   * @param rfid_tools: various RFID utilities
   * @param distance_log_path: where to save distance logs
   * @return the accuracy computed using the belief maps.
   */
  double findTags(vector<RFIDGridmap> *RFID_maps_list, vector<pair<double, double>> *tags_coord, dummy::Map *map, 
                string detection_log, string accuracy_log, 
                int initRange, long numConfiguration,
                RFID_tools *rfid_tools, string distance_log_path);

  /**
   * Save the RFID map as image on disk.
   * 
   * @param RFID_maps_list: list of RFID belief maps
   * @param root: the path where to save.
   */
  void saveRFIDMaps(vector<RFIDGridmap> *RFID_maps_list, string root);

  /**
   * Compute the size of the ellipse of the update (legacy).
   *
   * @param X_max: usually correspond to the sensor range, how far from the robot to scan
   * @param X_min: how back from the robot to scan
   * @param major_axis: major axis of the resulting ellipse
   * @param minor_axis: minor axis of the resulting ellipse
   */ 
  void getEllipseSize(int X_max, int X_min, double *major_axis, double *minor_axis);

  /**
   * Save the RFID map as image on disk with the ground truth overimpressed..
   * 
   * @param RFID_maps_list: list of RFID belief maps
   * @param tags_coord: coordinate of all the RFID tags.
   * @param root: the path where to save.
   */
  void saveRFIDMapsWithGroundTruths(vector<RFIDGridmap> *RFID_maps_list, vector<pair<double, double>> *tags_coord, string root);

  /**
   * Build a topological map of nodes and reachable ones using the list of cells explored in adanced.
   * 
   * @param astar: implementation of astar used for calculating distance between cells
   * @param record: utilities for managing poses
   * @param history: list of cells previously seen
   * @param map: reference to the map, needed for astar
   * 
   * @return the topological map modeled as list of cell and associated neighbors
   */
  std::vector<pair<Pose, list<Pose>>> getTopologicalMap(Astar *astar, EvaluationRecords *record, vector<string> *history, dummy::Map *map );

  std::vector<pair<pair<double, double>, list<Pose>>> getTopologicalMapSlim(Astar *astar, EvaluationRecords *record, vector<string> *history, dummy::Map *map );

  /**
   * Given a list of cells, find which one is the farer from a target one
   * @param nearCandidates: list of cells to analyse
   * @param target: the current cell of the robot
   * @return the farer cell from the current one
   */
  Pose getFarestNeighbor(list<Pose> *nearCandidates, Pose *target);

  void updateCriteria(double w_info_gain, double w_travel_distance, double w_sensing_time, double w_rfid_gain, double w_battery_status);

protected:

Pose invertedInitial, eastInitial, westInitial;
double w_info_gain, w_travel_distance, w_sensing_time, w_rfid_gain, w_battery_status;
int count_ = 0;

};


#endif
