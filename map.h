#ifndef MAP_H
#define MAP_H

#include <fstream> // ifstream
#include <sstream> // stringstream
#include <vector>
#include <string>
#include <iostream>
#include "pose.h"

using namespace std;
namespace dummy{

class Map
{

public:
  
  Map(std::ifstream& infile, int resolution);	//constructor, takes in binary map file .pgm
 // ~Map();					//destructor
  Map();
  void setGridValue(int value, int i, int j);	//setter for value in position ij of the grid
  int getGridValue(int i, int j) const;		//getter for value in position ij of the grid
  int getGridValue(int i) const;			//getter for value in position i of the grid vector
  int getMapValue(int i, int j);		//getter for value in position ij of the map
  int getNumGridRows() const;
  int getNumGridCols() const;
  int getNumRows();
  int getNumCols();
  void findFreeEdges(int cX, int cY);
  void addEdgePoint(int x, int y);
  //std::vector<int> getMap();
  //std::vector<int> getGrid();
  std::vector<vector<int> > getMap2D();
  std::vector<int> grid;			//vector containing the map as grid of cells sized 1 square metre
  int numGridRows;
  int numGridCols;
  Pose getRobotPosition();
  long getTotalFreeCells();
  
  
protected:
  std::vector<int> map;				//vector containing the original map as binary matrix (0 -> free, 1 -> obstacle)
  void createMap(std::ifstream& infile);
  void createGrid(int resolution);
  void createNewMap();
  int numRows;
  int numCols;
  std::vector<std::pair<int, int> > edgePoints;
  long totalFreeCells;
  
};
}

#endif
