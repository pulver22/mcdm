#include <vector>
#include "math.h"
#include "map.h"
#include "ray.h"

# define PI           3.14159265358979323846  /* pi */

void Ray::findCandidatePositions(const import_map::Map &map, int posX, int posY, int orientation, double FOV, int range)
{
  Ray::posX = posX;
  Ray::posY = posY;
  Ray::orientation = orientation;
  Ray::FOV = FOV;
  Ray::range = range;
  
  
  for(double phi = (PI/2 - FOV/2); phi < (PI/2 + FOV/2); phi += (FOV/32))		//range through the circular sector
  {
    int hit = 0;			//set to 1 when obstacle is hit by ray
    int u = 0;				//current distance of the ray
    double tempX, tempY;
    while(hit == 0 && u < range)	//scan the map along the ray until obstacle is found or end of range
    {
      
      tempX = mapX;
      tempY = mapY;
      
      //drives the ray according to orientation    X and Y are inverted in the formula in order to consider the euclidian space, not the array i(x) and j(y) indexes
      
      if(orientation == 0)
      {
      mapY = posY + u*sin(phi);
      mapX = posX + u*cos(phi);
      }
      
      if(orientation == 90)
      {
      mapY = posY + u*cos(phi);
      mapX = posX - u*sin(phi);
      }
      
      if(orientation == 180)
      {
      mapY = posY - u*sin(phi);
      mapX = posX - u*cos(phi);
      }
      
      if(orientation == 270)
      {
      mapY = posY - u*cos(phi);
      mapX = posX + u*sin(phi);
      }
      
      //rounding of the cell indexes	TO BE REFINED
      
      /*
      
      if (mapY > 0) mapY += 0.5;
      else mapY -= 0.5;
	
      if (mapX > 0) mapX += 0.5;
      else mapX -= 0.5;
      
      */
      
      //std::cout << mapY << " " << mapX << std::endl;
      
      if((map.getGridValue((int)mapX,(int)mapY) != 2) || (u + 1 == range && map.getGridValue((int)mapX,(int)mapY) == 2 )) 
      {
	hit = 1;
	int newposition = 1;
	std::pair<int, int> temp = std::make_pair<int, int>((int)tempX,(int)tempY);
	
	for(int i = 0; i < Ray::edgePoints.size(); ++i)
	{
	  if(Ray::edgePoints.at(i) == temp) newposition = 0;
	}
	
	if(newposition == 1)
	{
	  Ray::edgePoints.push_back(temp);
	}
      }
      ++u;
    }
  }
}

vector< int > Ray::getCandidatePositions()
{
  return Ray::edgePoints;
}

void Ray::setGrid(const import_map::Map &map)
{
  Ray::numGridCols = map.getNumGridCols();
  Ray::numGridRows = map.getNumGridRows();
  
  for (int i = 0; i < map.grid.size(); ++i)
  {
    Ray::grid.push_back(map.getGridValue(i));
  }
}

int Ray::getInformationGain(const import_map::Map &map, int posX, int posY, int orientation, double FOV, int range)
{
  setGrid(map);
  Ray::posX = posX;
  Ray::posY = posY;
  Ray::orientation = orientation;
  Ray::FOV = FOV;
  Ray::range = range;
  
  int counter = 0;			//count free cells that can be scanned
  
  for(double phi = (PI/2 - FOV/2); phi < (PI/2 + FOV/2); phi += (FOV/32))		//range through the circular sector
  {
    int hit = 0;			//set to 1 when obstacle is hit by ray
    int u = 0;				//current distance of the ray
    while(hit == 0 && u < range)	//scan the map along the ray until obstacle is found or end of range
    {
      
      //drives the ray according to orientation    X and Y are inverted in the formula in order to consider the euclidian space, not the array i(x) and j(y) indexes
      
      if(orientation == 0)
      {
      mapY = posY + u*sin(phi);
      mapX = posX + u*cos(phi);
      }
      
      if(orientation == 90)
      {
      mapY = posY + u*cos(phi);
      mapX = posX - u*sin(phi);
      }
      
      if(orientation == 180)
      {
      mapY = posY - u*sin(phi);
      mapX = posX - u*cos(phi);
      }
      
      if(orientation == 270)
      {
      mapY = posY - u*cos(phi);
      mapX = posX + u*sin(phi);
      }
      
      //rounding of the cell indexes
      
      if (mapY > 0) mapY += 0.5;
      else mapY -= 0.5;
	
      if (mapX > 0) mapX += 0.5;
      else mapX -= 0.5;
      
      std::cout << mapY << " " << mapX << std::endl;
      
      if(Ray::grid.at > 0) hit = 1;		//hit set to 1 if an obstacle is found
      if(Ray::getGridValue((int)mapX, (int)mapY) == 0)			//free cell found
      {
	Ray::setGridValue((int)mapX, (int)mapY, -1);			//set the cell to -1 so it won't be counted again in the future
	counter++;						//increase the count of free cells
      }
      ++u;							//move forward with the ray
    }
    return counter;
  }
}
  

void Ray::setGridValue(int i, int j, int value)
{
  //if(value == 0 || value == 1 || value == 2)
  {
  Ray::grid[i*numGridCols + j] = value;
  }
}

int Ray::getGridValue(int i, int j)
{
  return Ray::grid[i*numGridCols + j];
}



