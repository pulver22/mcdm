#include <vector>
#include "math.h"
#include "map.h"
#include "ray.h"

# define PI           3.14159265358979323846  /* pi */

namespace dummy{
  
Ray::Ray()
{

}

//Check if a cell is candidate position: return 1 if the cell is adjacent to at least one free cell, 0 otherwise
int Ray::isCandidate(const dummy::Map* map, int i, int j)
{
  int candidate = 0;
  int r = i;
  int s = j;
  int minR = r - 1, maxR = r + 1, minS = s -1, maxS = s + 1;
  if(minR < 0) minR = 0;
  if(minS < 0) minS = 0;
  if(maxR > map->getNumGridRows()) maxR = map->getNumGridRows();
  if(maxS > map->getNumGridCols()) maxS = map->getNumGridCols();
  
  
  for(r = minR; r <= maxR; ++r)
  {
    for(s = minS; s <= maxS; ++s)
    {
      if (map->getGridValue(r, s) == 0) candidate = 1;
    }
  }
  return candidate;
  
}


//finds the candidate positions: cells already scanned in range of the robot which are adjacent to at least one free cell
void Ray::findCandidatePositions(Map* map, int posX, int posY, int orientation, double FOV, int range)
{
  Ray::numGridRows = map->getNumGridRows();
  
  //set the correct FOV orientation
  double startingPhi = orientation*PI/180 - FOV/2;	
  double endingPhi = orientation*PI/180 + FOV/2;
  int add2pi = 0;
  
  if(startingPhi <= 0) 
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }
  
  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;
    
  //select the portion of map to be scanned
  int minI = posX - range;
  int maxI = posX + range;
  int minJ = posY - range;
  int maxJ = posY + range;
  
  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();
  
  //scan the cells in the selected portion of the map
  for(int i = minI; i <= maxI; ++i)
  {
    for(int j = minJ; j <=maxJ; ++j)
    {
      
      double distance = sqrt((i - posX)*(i - posX) + (j - posY)*(j - posY));
      
      //if a cell is a candidate one and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 2 && distance <= range)
      {

	if(Ray::isCandidate(map, i, j) == 1)
	{
	
	double curX = posX;		//starting position of the ray
	double curY = posY;
	double robotX = posX;		//position of the robot
	double robotY = posY;
	
	double convertedI = Ray::convertPoint(i);
	double convertedRX = Ray::convertPoint(robotX);

	double slope = atan2(Ray::convertPoint(i) - Ray::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2
	
	if(slope <= 0 && add2pi == 0) slope = slope + 2*PI; 
	if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values
	
	//std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

	if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
	{
	  //raycounter++;
	  //std::cout << "Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;
	  
	  int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
	  double u = 0;			//current position along the ray
	  while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
	  {
	    
	    //convert the position on the ray to cell coordinates to check the grid
	    curY = robotY + 0.5 + u*cos(slope);
	    curX = robotX + 0.5 - u*sin(slope);
	    
	    //not needed, but left anyway
	    if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols()) hit = 1;        
      
	    if(map->getGridValue((int)curX, (int)curY) == 1) 
	    {
	      hit = 1;		//hit set to 1 if an obstacle is found
	      //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
	    }

	    if((int)curX == i && (int)curY == j)	//if the free cell is reached, save it as edge point and stop the ray.
	    {
	      std::pair<int, int> temp = std::make_pair<int, int>(i, j);
	      Ray::edgePoints.push_back(temp);
	      //std::cout << "Cell scanned: " << (int)curY << " " << (int)curX << std::endl;
	      hit = 1;
	    }
	    u += 0.2;		//move forward along the ray
	  }
	}
	}
      }
    }
  }
}


vector< std::pair<int, int> > Ray::getCandidatePositions()
{
  return Ray::edgePoints;
}

void Ray::emptyCandidatePositions()
{
  edgePoints.clear();
}


//calculate the sensing time of a possible scanning operation, returns the minimum FOV required to scan all the free cells from the considered pose
//ATTENTION: the FOV is always centered in the orientation of the robot
//ATTENTION: in order to optimize the computing time, this method should be fused with the information gain one
double Ray::getSensingTime(const Map* map, int posX, int posY, int orientation, double FOV, int range)
{
  Ray::numGridRows = map->getNumGridRows();
  
  double minPhi = 0;	//slope of the first ray required
  double maxPhi = 0;	//slope of the last ray required
  int phiFound = 0;	//set to 1 if at least a cell can be scanned
  
  //set the correct FOV orientation
  double startingPhi = orientation*PI/180 - FOV/2;	
  double endingPhi = orientation*PI/180 + FOV/2;
  int add2pi = 0;
  
  if(startingPhi <= 0) 
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }
  
  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;
    
  //select the portion of map to be scanned
  int minI = posX - range;
  int maxI = posX + range;
  int minJ = posY - range;
  int maxJ = posY + range;
  
  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();
  
  //scan the cells in the selected portion of the map
  for(int i = minI; i <= maxI; ++i)
  {
    for(int j = minJ; j <=maxJ; ++j)
    {
      
      double distance = sqrt((i - posX)*(i - posX) + (j - posY)*(j - posY));
      
      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range)
      {
	double curX = posX;		//starting position of the ray
	double curY = posY;
	double robotX = posX;		//position of the robot
	double robotY = posY;
	
	double convertedI = Ray::convertPoint(i);
	double convertedRX = Ray::convertPoint(robotX);

	double slope = atan2(Ray::convertPoint(i) - Ray::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2
	
	if(slope <= 0 && add2pi == 0) slope = slope + 2*PI; 
	if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values
	
	//std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

	if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
	{
	  //std::cout << "Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;
	  
	  int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
	  double u = 0;			//current position along the ray
	  while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
	  {
	    
	    //convert the position on the ray to cell coordinates to check the grid
	    curY = robotY + 0.5 + u*cos(slope);
	    curX = robotX + 0.5 - u*sin(slope);
	    
	    //not needed, but left anyway
	    if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols()) hit = 1;        
      
	    if(map->getGridValue((int)curX, (int)curY) == 1) 
	    {
	      hit = 1;		//hit set to 1 if an obstacle is found
	      //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
	    }

	    if((int)curX == i && (int)curY == j)	//free cell reached, check if the slope is a candidate for first or last ray
	    {
	      if(phiFound == 0)		//enters if it is the first free cell found
	      {
		phiFound = 1;
		minPhi = slope;
		maxPhi = slope;
	      }
	      if(phiFound == 1)
	      {
		if(slope < minPhi) minPhi = slope;
		if(slope > maxPhi) maxPhi = slope;
	      }
	      
	      hit = 1;
	    }
	    u += 0.2;		//move forward along the ray
	  }
	}
      }
    }
  }
  double value;		//FOV to return
  
  if(phiFound == 0) return -1;		//return -1 if no free cells can be scanned
  else 					//return the correct FOV (ALWAYS CENTERED ON THE ORIENTATION)
  { 
    if(minPhi - startingPhi <= endingPhi - maxPhi) value = (endingPhi - startingPhi - 2*(minPhi - startingPhi));
	else value = (endingPhi - startingPhi - 2*(endingPhi - maxPhi));
  }
  
  //std::cout << "startingPhi " << startingPhi << " endingPhi " << endingPhi << " minPhi " << minPhi << " maxPhi " << maxPhi << std::endl;
  
  return value;
}


//perform the sensing operation by setting the value of the free cell scanned to 2
void dummy::Ray::performSensingOperation(dummy::Map *map, int posX, int posY, int orientation, double FOV, int range)
{
  Ray::numGridRows = map->getNumGridRows();
  
  //set the correct FOV orientation
  double startingPhi = orientation*PI/180 - FOV/2;	
  double endingPhi = orientation*PI/180 + FOV/2;
  int add2pi = 0;
  
  if(startingPhi <= 0) 
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }
  
  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;
    
  //select the portion of map to be scanned
  int minI = posX - range;
  int maxI = posX + range;
  int minJ = posY - range;
  int maxJ = posY + range;
  
  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();
  
  //scan the cells in the selected portion of the map
  for(int i = minI; i <= maxI; ++i)
  {
    for(int j = minJ; j <=maxJ; ++j)
    {
      
      double distance = sqrt((i - posX)*(i - posX) + (j - posY)*(j - posY));
      
      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range)
      {
	double curX = posX;		//starting position of the ray
	double curY = posY;
	double robotX = posX;		//position of the robot
	double robotY = posY;
	
	double convertedI = Ray::convertPoint(i);
	double convertedRX = Ray::convertPoint(robotX);

	double slope = atan2(Ray::convertPoint(i) - Ray::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2
	
	if(slope <= 0 && add2pi == 0) slope = slope + 2*PI; 
	if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values
	
	//std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

	if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
	{
	  //raycounter++;
	  //std::cout << "Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;
	  
	  int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
	  double u = 0;			//current position along the ray
	  while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
	  {
	    
	    //convert the position on the ray to cell coordinates to check the grid
	    curY = robotY + 0.5 + u*cos(slope);
	    curX = robotX + 0.5 - u*sin(slope);
	    
	    //not needed, but left anyway
	    if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols()) hit = 1;        
      
	    if(map->getGridValue((int)curX, (int)curY) == 1) 
	    {
	      hit = 1;		//hit set to 1 if an obstacle is found
	      //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
	    }

	    if((int)curX == i && (int)curY == j)	//if the free cell is reached, set its value to 2 and stop the ray
	    {
	      map->setGridValue(2, i, j);
	      //std::cout << "Cell scanned: " << (int)curY << " " << (int)curX << std::endl;
	      hit = 1;
	    }
	    u += 0.2;		//move forward along the ray
	  }
	}
      }
    }
  }
}

//convert the value along the y axis to the cartesian space in order to compute atan2
int dummy::Ray::convertPoint(int y)
{
  return (Ray::numGridRows - 1 - y);
}


int dummy::Ray::getInformationGain(const dummy::Map* map, int posX, int posY, int orientation, double FOV, int range)
{
  //int raycounter = 0;
  int counter = 0;	//count number of free cells that can be seen
  Ray::numGridRows = map->getNumGridRows();
  
  //set the correct FOV orientation
  double startingPhi = orientation*PI/180 - FOV/2;	
  double endingPhi = orientation*PI/180 + FOV/2;
  int add2pi = 0;
  
  if(startingPhi <= 0) 
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }
  
  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;
    
  //select the portion of map to be scanned
  int minI = posX - range;
  int maxI = posX + range;
  int minJ = posY - range;
  int maxJ = posY + range;
  
  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();
  
  //scan the cells in the selected portion of the map
  for(int i = minI; i <= maxI; ++i)
  {
    for(int j = minJ; j <=maxJ; ++j)
    {
      
      double distance = sqrt((i - posX)*(i - posX) + (j - posY)*(j - posY));
      
      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range)
      {
	double curX = posX;		//starting position of the ray
	double curY = posY;
	double robotX = posX;		//position of the robot
	double robotY = posY;
	
	double convertedI = Ray::convertPoint(i);
	double convertedRX = Ray::convertPoint(robotX);

	double slope = atan2(Ray::convertPoint(i) - Ray::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2
	
	if(slope <= 0 && add2pi == 0) slope = slope + 2*PI; 
	if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values
	
	//std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

	if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
	{
	  //raycounter++;
	  //std::cout << "Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;
	  
	  int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
	  double u = 0;			//current position along the ray
	  while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
	  {
	    
	    //convert the position on the ray to cell coordinates to check the grid
	    curY = robotY + 0.5 + u*cos(slope);
	    curX = robotX + 0.5 - u*sin(slope);
	    
	    //not needed, but left anyway
	    if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols()) hit = 1;        
      
	    if(map->getGridValue((int)curX, (int)curY) == 1) 
	    {
	      hit = 1;		//hit set to 1 if an obstacle is found
	      //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
	    }

	    if((int)curX == i && (int)curY == j)	//if the free cell is reached, increase counter and stop the ray.
	    {
	      ++counter;
	      //std::cout << "Cell scanned: " << (int)curY << " " << (int)curX << std::endl;
	      hit = 1;
	    }
	    u += 0.2;		//move forward along the ray
	  }
	}
      }
    }
  }
  //std::cout << "Number of rays: " << raycounter << std::endl;
  return counter;	//return the number of free cells
}


}

