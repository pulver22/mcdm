#ifndef RAY_H
#define RAY_H

#include <vector>
#include "math.h"
#include "map.h"


using namespace dummy;
class Ray
{
  
public:
  
  Ray();
  //~Ray();
  void findCandidatePositions(dummy::Map *map, int posX, int posY, int orientation, double FOV, int range);
  std::vector<std::pair<int, int> > getCandidatePositions();
  void setGrid(const dummy::Map* map);
  int getInformationGain(const dummy::Map* map, int posX, int posY, int orientation, double FOV, int range);
  void performSensingOperation(dummy::Map *map, int posX, int posY, int orientation, double FOV, int range);
  int getGridValue(int i, int j);
  void empyCandidatePositions();
  
private:
  double mapX, mapY;			//coordinates in the map
  int posX, posY;		//starting position of the robot
  int orientation;			//orientation of the robot (0, 90, 180, 270 degrees)
  double FOV;
  int range;			//range of the scanner
  std::vector<std::pair<int, int> > edgePoints;
  std::vector<int> grid;
  void setGridValue(int i, int j, int value);
  int numGridCols;
  int numGridRows;
};


#endif