#ifndef RAY_H
#define RAY_H

#include <vector>
#include "math.h"
#include "map.h"


class Ray
{
  
public:
  
  Ray();
  ~Ray();
  void findCandidatePositions(const import_map::Map &map, int posX, int posY, int orientation, double FOV, int range);
  std::vector<int> getCandidatePositions();
  void setGrid(const import_map::Map &map);
  int getInformationGain(const import_map::Map &map, int posX, int posY, int orientation, double FOV, int range);
  
private:
  double mapX, mapY;			//coordinates in the map
  int posX, posY;		//starting position of the robot
  int orientation;			//orientation of the robot (0, 90, 180, 270 degrees)
  double FOV;
  int range;			//range of the scanner
  std::vector<std::pair<int, int> > edgePoints;
  std::vector<int> grid;
  int getGridValue(int i, int j);
  void setGridValue(int i, int j, int value);
  int numGridCols;
  int numGridRows;
};

#endif