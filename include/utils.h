#ifndef UTILS_H
#define UTILS_H

#include "pose.h"
#include "mcdmfunction.h"
#include "evaluationrecords.h"
#include "PathFinding/astar.h"
#include "newray.h"


using namespace dummy;



class Utilities
{
public:
  Utilities();
  ~Utilities();
  /**
  * Check if a Pose object is already present within a list
  * @param list: the list of Pose objects
  * @param p: the Pose object to look for
  */
  bool contains ( std::list< Pose >& list, Pose& p );
  /**
  * Remove a Pose object from a list of Pose objects
  * @param possibleDestinations: list of Pose objects
  * @param p: the object to removeS
  */
  void cleanPossibleDestination2(std::list<Pose> *possibleDestinations, Pose &p);
  /**
  * Calculate the frontiers from the starting position and add it to the graph structure
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
  void pushInitialPositions (dummy::Map map, int x, int y, int orientation,
                             int range, int FOV, double threshold,
                             string actualPose,
                             vector< pair< string, list< Pose > > > *graph2,
                             MCDMFunction *function);
  /**
  * Calculate the time required for performing a scan with the TDLAS sensor
  * @param scanAngle: the angle to scan
  */
  double calculateScanTime ( double scanAngle );
  /**
  * Calculate the length of a path
  * @param list: a list of Pose lying on the path
  * @param map: a copy of the map
  * @param astar: the astar object used for parsing the map and calculating the distance
  */
  void calculateDistance(list<Pose> list, dummy::Map& map, Astar* astar);
  /**
  * Create a new Pose object starting from another one
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
  */
  void updatePathMetrics(int* count, Pose* target, Pose* previous, string actualPose,
                         list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                         dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList,
                         vector<string>* history, int encodedKeyValue, Astar* astar ,
                         long* numConfiguration, double* totalAngle,
                         double * travelledDistance, int* numOfTurning , double scanAngle);
  /**
  * Get the list of Pose assumed only once along the traversed path
  * @param history: the list of Pose assumed by the robot one or more times (during bakctracking)
  * @param record: evaluationRecords object required for converting a string to a Pose
  */
  list<Pose> cleanHistory(vector<string>* history, EvaluationRecords* record);
  /**
  * Print in console a summary of the exploration task
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
                   long numConfiguration, double travelledDistance,
                   int numOfTurning, double totalAngle, double totalScanTime);
  /**
  * Save on file the exploration results
  * @param name: the output file name
  * @param content: the sring to append to the file
  * @param append: if the file needs to be created from scratch or we append
  */
  void filePutContents(const std::string& name, const std::string& content,
                       bool append = false);
protected:

};


#endif
