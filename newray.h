#ifndef NEWRAY_H
#define NEWRAY_H

#include <vector>
#include "math.h"
#include "map.h"

using namespace dummy;

class NewRay
{
  
public:
  
  NewRay();
  //~Ray();
  void findCandidatePositions(dummy::Map &map, int posX, int posY, int orientation, double FOV, int range);
  int isCandidate(const dummy::Map &map, int i, int j);
  std::vector<std::pair<int, int> > getCandidatePositions();
  double getSensingTime(const dummy::Map &map, int posX, int posY, int orientation, double FOV, int range);
  void performSensingOperation(dummy::Map &map, int posX, int posY, int orientation, double FOV, int range);
  void emptyCandidatePositions();
  int getInformationGain(const dummy::Map &map, int posX, int posY, int orientation, double FOV, int range);
  int convertPoint(int y);
  
protected:
  double mapX, mapY;			//coordinates in the map
  int posX, posY;		//starting position of the robot
  int orientation;			//orientation of the robot (0, 90, 180, 270 degrees)
  double FOV;
  int range;			//range of the scanner
  std::vector<std::pair<int, int> > edgePoints;
  int numGridCols;
  int numGridRows;
};


#endif