#ifndef NEWRAY_H
#define NEWRAY_H

#include "map.h"
#include "math.h"
#include <stdio.h>
#include <utility>
#include <vector>

using namespace dummy;

class NewRay {

public:
  /**
   * Constructor
   */
  NewRay();

  /**
   * Destructor
   */
  ~NewRay();

  /**
   * Find the candidate position starting from a reference cell.
   * Candidate position are cells already scanned in range of the robot which
   * are adjacent to at least one free cell
   *
   * @param map: the map used
   * @param posX: the x-position of the robot
   * @param posY: the y-position of the robot
   * @param orientation: the orientation of the robot
   * (0,45,90,135,180,225,270,325,360)
   * @param FOV: the FOV of the sensor used for scanning
   * @param range: the range of the sensor used for scanning
   */
  void findCandidatePositions(dummy::Map *map, long posX, long posY,
                              int orientation, double FOV, int range);

  /**
   * Find the candidate position starting from a reference cell.
   * Candidate position are cells already scanned in range of the robot which
   * are adjacent to at least one free cell
   *
   * @param map: the map used
   * @param posX: the x-position of the robot
   * @param posY: the y-position of the robot
   * @param orientation: the orientation of the robot
   * (0,45,90,135,180,225,270,325,360)
   * @param FOV: the FOV of the sensor used for scanning
   * @param range: the range of the sensor used for scanning
   */
  void findCandidatePositions2(dummy::Map *map, long posX, long posY,
                               int orientation, double FOV, int range);

  /**
   * Check if a cell is candidate position: return 1 if the cell is adjacent to
   * at least one free cell, 0 otherwise
   *
   * @param map: the map used
   * @param i: the x-position of the cell
   * @param j: the y-position of the cell
   * @return if a cell is candidate (1, meaning it's adjacent to a free cell) or
   * not (0)
   */
  int isCandidate(const dummy::Map *map, long i, long j);

  /**
   * Check if a cell is candidate position on the PathPlanningGrid
   * @param map: the map used (the pathplanningGrid)
   * @param i: the x-position of the cell
   * @param j: the y-position of the cell
   * @return if a cell is candidate (1, meaning it's adiacent to a free cell) or
   * not (0)
   */
  int isCandidate2(const dummy::Map *map, long i, long j);

  /**
   * Get the list of candidate position
   *
   * @return the list of candidate position on the frontier
   */
  std::vector<std::pair<long, long>> getCandidatePositions();

  /**calculate the sensing time of a possible scanning operation
   * ATTENTION: the FOV is always centered in the orientation of the robot
   * ATTENTION: in order to optimize the computing time, this method should be
   * fused with the information gain one
   *
   * @param map: the reference to the map
   * @param posX: the current x-coord of the robot
   * @param posY: the current y-coord of the robot
   * @param orientation: the current orientation of the robot
   * @param FOV: the sensor FOV
   * @param range: the sensor range
   * @return the minimum FOV required to scan all the free cells from the
   * considered pose
   */
  pair<double, double> getSensingTime(const dummy::Map *map, long int posX,
                                      long int posY, int orientation,
                                      double FOV, int range);

  /**
   * Perform the sensing operation by setting the value of the free cell 
   * scanned to 2 inside an ellipsoid
   *
   * @param map: the reference to the map
   * @param posX: the current x-coord of the robot
   * @param posY: the current y-coord of the robot
   * @param posOri: the current orientation of the robot
   * @param firstAngle: the starting scanning angle
   * @param lastAnge: the ending scanning angle
   * @param a_pcell: ellipse radius on the x-axis in planning cells
   * @param b_pcell: ellipse radius on the y-axis in planning cells
   * @param debug:
   * @return the number of scanned cells
   */
  int performSensingOperationEllipse(dummy::Map *map, long posX, long posY,
                                     int posOri, double firstAngle,
                                     double lastAngle, long a_pcell,
                                     long b_pcell, bool debug = false);
  
  /**
   * Perform the sensing operation by setting the value of the free cell scanned to 2
   * @param map: the reference to the map
   * @param posX: x-position of the robot
   * @param posY: y-position of the robot
   * @param orientation: orientation of the robot (0,45,90,135,180,225,270,325,360)
   * @param FOV: the FOV of the sensor used
   * @param range: the range of the sensor used
   * @param firstAngle: the ange from which start to sense
   * @param lastAngle: the final angle of the sensing operation
   * @return the number of scanned cells
   */
  int performSensingOperation(dummy::Map *map, long posX, long posY,
                              int orientation, double FOV, int range,
                              double firstAngle, double lastAngle);

  /**
   * Clear the list of candidate positions
   */
  void emptyCandidatePositions();

  /**
   * Calculate how many free cells are within the scannable area
   * 
   * @param map: the map to be scanned
   * @param posX: the x-position of the robot
   * @param posY: the y-position of the robot
   * @param orientation: the orientation of the robot (0,45,90,135,180,225,270,325,360)
   * @param FOV: the FOV of the sensor used for scanning
   * @param range: the range of the sensor used for scanning
   * @return the number of the free scell in the scannable area
   */
  int getInformationGain(const dummy::Map *map, long posX, long posY,
                         int orientation, double FOV, int range);

  /**
   * Calculate how many free cells are within the scannable area
   * 
   * @param map: the map to be scanned
   * @param posX: the x-position of the robot
   * @param posY: the y-position of the robot
   * @param orientation: the orientation of the robot (0,45,90,135,180,225,270,325,360)
   * @param FOV: the FOV of the sensor used for scanning
   * @param range: the range of the sensor used for scanning
   * @return the number of the free scell in the scannable area
   */
  int getRFIDGain(const dummy::Map *map, long posX, long posY, int orientation,
                  double FOV, int range);

  /**
   Convert the value along the y axis to the cartesian space in order to compute atan2
   * @param y: the y-coord of the robot
   * @return the new coordinate in the map frame
   */
  long convertPoint(long y);

  /**
   *Convert the value along the y axis to the cartesian space of the PathPLanningGrid in order to compute atan2
   * 
   * @param y: the y-coord of the robot
   * @return the new coordinate in the pathplanning grid frame
   */
  long convertPointPP(long y);
  
  /**
   * Set the ratio between the two grids
   * 
   * @param value: the ratio between the cell'size of the two grids
   * @return nothing
   */
  int setGridToPathGridScale(int value);

  /**
   * Perform the sensing operation by setting the value of the free cell scanned to 2
   * @param map
   * @param posX: x-position of the robot
   * @param posY: y-position of the robot
   * @param orientation: orientation of the robot (0,45,90,135,180,225,270,325,360)
   * @param FOV: the FOV of the sensor used
   * @param range: the range of the sensor used
   * @param firstAngle: the ange from which start to sense
   * @param lastAngle: the final angle of the sensing operation
   * @return
   */
  void performRFIDSensingOperation(dummy::Map *map, long posX, long posY,
                                   int orientation, double FOV, int range,
                                   double power, double firstAngle,
                                   double lastAngle);

protected:
  double mapX, mapY; // coordinates in the map
  long posX, posY;   // starting position of the robot
  int orientation;   // orientation of the robot (0, 90, 180, 270 degrees)
  double FOV;
  int range; // range of the scanner
  std::vector<std::pair<long, long>> edgePoints;
  long numGridCols;
  long numGridRows;
  long numPathPlanningGridRows;
  long numPathPlanningGridCols;
  long informationGain;
  double sensingTime;
  int gridToPathGridScale;
};

#endif
