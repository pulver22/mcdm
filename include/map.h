#ifndef MAP_H
#define MAP_H

#include "RFIDGridmap.h"
#include "pose.h"
#include <fstream> // ifstream
#include <iostream>
#include <sstream> // stringstream
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;
namespace dummy {

class Map {

public:
  /**
   * Constructor
   *
   * @param infile: the path used for reading the map
   * @param fileURI: map file absolute path and filename
   * @param resolution: the resolution of the map
   * @param imgresolution: the resolution for building the planningGrid
   */
  Map(std::ifstream &infile, double resolution,
      double imgresolution); // constructor, takes in binary map file .pgm
                             // Map(nav_msgs::OccupancyGrid ros_msg);
  //  ~Map();					//destructor

  /**
   * @brief Map::Map empty constructore
   */
  Map();

  /**
   * Set a value for a cell in the Grid
   * 
   * @param value: 0 -> unscanned free cell, 1 -> obstacle cell, 2 -> scanned free cell
   * @param i: the x-position(row) in the grid
   * @param j: the y-position(column) in the grid
   */
  void setGridValue(int value, long i,
                    long j); // setter for value in position ij of the grid
  
  /**
   * Get the value associated with one cell of the grid
   * 
   * @param i: the x-position in the grid
   * @param j: the y-position in the grid
   * @return the associated value with that cell
   */
  int getGridValue(long i, long j) const; 

  /**
   * Get the value of a cell in the gridmap
   * 
   * @param i: the index of the cell
   * @return the value contained in the cell
   */
  long getGridValue(long i) const;
  
  /**
   * Get the value of one cell of the map
   * 
   * @param i: the x-position in the map
   * @param j: the y-position in the map
   * @return the value associated with that cell
   */
  long getMapValue(long i, long j);

  /**
   * Get the number of rows of the grid
   * 
   * @return the number of rows of the grid
   */
  long getNumGridRows() const;
  
  /**
   * Get the number of columns of the grid
   * 
   * @return the number of columns of the grid
   */
  long getNumGridCols() const;

  /**
   * Get the number of rows of the map
   * 
   * @return the number of rows of the map
   */
  long getNumRows();

  /**
   * Get the number of columns of the map
   * 
   * @return the number of columns of the map
   */
  long getNumCols();
  void findFreeEdges(int cX, int cY);

  /**
   * Create a point and add it to list of cells on the edge of the scanned area
   * 
   * @param x: the x-coord of the cell
   * @param y: the y-cord of the cell
   */
  void addEdgePoint(int x, int y);
  // std::vector<int> getMap();
  // std::vector<int> getGrid();

  /**
   * Create a 2D map from a vector one
   * 
   * @return the 2D map
   */
  std::vector<vector<long>> getMap2D();

  /**
   * Get the current pose of the robot
   * 
   * @return the current pose of the robot
   */
  Pose getRobotPosition();

  /**
   * Calculate the number of free cells in the map
   * 
   * @return the number of free cells
   */
  long getTotalFreeCells();

  /**
   * Set the pose of the robot in the map
   * 
   * @param p the current pose of the robot
   */
  void setCurrentPose(Pose &p);

  /**
   * Save on disk an image of the map
   */
  void drawVisitedCells();

  /**
   * Save on disk the list of all the cells visited by the robot
   * 
   * @param history: the list of visited cells
   */
  void printVisitedCells(vector<string> &history);

  /**
   * Get the current value of a cell in the planning grid
   * 
   * @param i: the x-position(row) in the planning grid
   * @param j: the y-position(column) in the planning grid
   * @return the value in that cell value: 0 -> unscanned free cell, 1 -> obstacle cell, 2 -> scanned free cell)
   */
  int getPathPlanningGridValue(long i, long j) const;

  /**
   * Assign a value to a cell in the planning grid
   * 
   * @param value:  0 -> unscanned free cell, 1 -> obstacle cell, 2 -> scanned free cell
   * @param i: the x-position(row) of the cell in the planning grid
   * @param j: the y-position(column) of the cell in the planning grid
   */
  void setPathPlanningGridValue(int value, int i, int j);

  /**
   * Get the number of columns in the planning grid
   * 
   * @return the number of columns in the planning grid
   */
  int getPathPlanningNumCols() const;

  /**
   * Get the number of rows in the planning grid
   * @return the number of rows in the planning grid
   */
  int getPathPlanningNumRows() const;

  /**
   * Get the scale(ratio) between the cell size in the grid and in that used for planning
   * 
   * @return the scale value
   */
  int getGridToPathGridScale() const;

  /**
   * Update the pathplanning grid and the rfid grid.
   * Check the navigation map and if the cells are scanned there, update the
   * corresponding one also in the other grids.
   *
   * @param x: the current x-coord of the robot
   * @param y: the current y-coord of the robot
   * @param rangeInMeters: the sensor range in meters
   * @param power: the transmitted power
   */
  void updatePathPlanningGrid(int x, int y, int rangeInMeters, double power);
  
  /**
   * Calculate the relative position of the RFID tag wrt the antenna
   * 
   * @param absTagX: absolute x-position of the RFID tag
   * @param absTagY: absolute x-position of the RFID tag
   * @param antennaX: absolute x-position of the antenna
   * @param antennaY: absolute y-position of the antenna
   * @return a pair containing the relative position of the tag to the antenna
   */
  std::pair<int, int> getRelativeTagCoord(int absTagX, int absTagY,
                                          int antennaX, int antennaY);
  
  /**
   * Update the grid cell summing the current value with a new reading
   * 
   * @param power: the sensed power by the antenna
   * @param i: the x-position in the grid
   * @param j: the y-position in the grid
   */
  void setRFIDGridValue(float power, int i, int j);

  /**
   * Save on disk an image representing the scanned environment. Lighter zone represents
   * higher probability for the presence of the RFID tag
   */
  void drawRFIDScan();

  /**
   * Save on disk an image representing the scanned environment. Lighter zone represents
   * higher probability for the presence of the RFID tag
   */
  void drawRFIDGridScan(RFIDGridmap grid);

  /**
   * Get the value(power) associated with one cell of the RFIDgrid
   * 
   * @param i: the x-position in the RFID grid
   * @param j: the y-position in the RFID grid
   * @return the associated value with that cell
   */
  int getRFIDGridValue(long i, long j) const;
  
  /**
   * Find the tag position looking at the submap with the highest cumulative pixel intensity
   * 
   * @return the coordinate of the tag
   */
  std::pair<int, int> findTag();

  /**
   * Find the tag position looking at the submap with the highest cumulative pixel intensity
   * 
   * @param grid: the RFID map associated with one tag
   * @return the total power in the submap and the coordinate of the tag
   */
  std::pair<int, std::pair<int, int>> findTagfromGridMap(RFIDGridmap grid);
  
  /**
   * Sample a random cell among the list of the free ones.
   * 
   * @return the coordinate of the selected cell
   */
  std::pair<long, long> getRandomFreeCell();
  // nav_msgs::OccupancyGrid toROSMsg();

  std::vector<long> grid; // vector containing the map as grid of cells sized 1 square metre
  std::vector<int> pathPlanningGrid;
  std::vector<int> RFIDGrid;
  long numGridRows;
  long numGridCols;
  int gridToPathGridScale;

protected:
  
  /**
   * Create a monodimensional vector containing the map
   *
   * @param infile: the path for reading the map
   */
  void createMap(std::ifstream &infile);

  /**
   * Create the map as a monodimension vector with 0 and 1
   *
   * @param resolution: the size of the cell's side
   */
  void createGrid(double resolution);

  /**
   * Map::createPathPlanningGrid Create the gridmap for path planning and for
   * radio sensing (both are identical)
   * @param  resolution: the size of the side of a cell
   * @return          nothing
   */
  void createPathPlanningGrid(double resolution);
  //  void createRFIDGrid(double resolution);

  /**
   *Save on disk an image representing the map of the environment and a list of
   *free cells in the map
   */
  void createNewMap();
  
  /** 
   * Decrease the number of the cells that still must be covered
 */
  void decreaseFreeCells();

  std::vector<long> map;
  int numPathPlanningGridRows;
  int numPathPlanningGridCols;
  long numRows;
  long numCols;
  std::vector<std::pair<int, int>> edgePoints;
  long totalFreeCells;
  Pose currentPose;
  std::vector<pair<long, long>> listFreeCells;
};
} 

#endif
