#ifndef MAP_H
#define MAP_H

#include <fstream> // ifstream
#include <sstream> // stringstream
#include <vector>
#include <string>
#include <iostream>
#include <utility>
#include "pose.h"
using namespace std;
namespace import_map{

class Map
{

public:
  
  Map(std::ifstream& infile, int resolution);	//constructor, takes in binary map file .pgm
 // ~Map();					//destructor
  void setGridValue(int value, int i, int j);	//setter for value in position ij of the grid
  int getGridValue(int i, int j);		//getter for value in position ij of the grid
  int getGridValue(int i);			//getter for value in position i of the grid vector
  int getMapValue(int i, int j);		//getter for value in position ij of the map
  int getNumGridRows();
  int getNumGridCols();
  int getNumRows();
  int getNumCols();
  Pose getRobotPosition();
  void findFreeEdges(int cX, int cY);
  void addEdgePoint(int x, int y);
  std::vector<int> getMap();
  std::vector<int> getGrid();
  std::vector<vector<int> > getMap2D();
  
  
protected:
  std::vector<int> map;				//vector containing the original map as binary matrix (0 -> free, 1 -> obstacle)
  std::vector<int> grid;			//vector containing the map as grid of cells sized 1 square metre
  void createMap(std::ifstream& infile);
  void createGrid(int resolution);
  int numGridRows;
  int numGridCols;
  int numRows;
  int numCols;
  std::vector<std::pair<int, int> > edgePoints;
  
};
}

#endif
