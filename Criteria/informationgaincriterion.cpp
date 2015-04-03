#include "informationgaincriterion.h"
#include "criteriaName.h"
#include <math.h>


InformationGainCriterion::InformationGainCriterion(double weight) :
    Criterion(INFORMATION_GAIN, weight, true)
{
}


InformationGainCriterion::~InformationGainCriterion()
{
}

double InformationGainCriterion::evaluate(Pose p, Map map)
{
    int px = p.getX();
    int py = p.getY();
    float resolution = map.map2D.getResolution();
    char orientation = p.getOrientation();
    int minSensedX, maxSensedX;
    int minSensedY,maxSensedY;
    
    int maxValueY = map.map2D.size();
    int maxValueX = map.map2D[0].size();
    
    //area contained in the sensor cone
    int sensedArea;
    //area occupied by obstacles in the sensor cone
    int occupiedArea;
    //effective information gain
    int unExploredMap;
    
    
    //calculate the area of the circular sector and then divide it by the map resolution to get the number of cell in it
    sensedArea = (int) 1/2 * pow(p.getRadius(),2) * p.getGamma();
    sensedArea = (int) sensedArea / map.map2D.getResolution();
    
    //calcuate the sensed map based on the robot orientation
    if( orientation == 'N'){
	minSensedX = px - p.getRadius() * cos(p.getGamma()/2);
	maxSensedX = px + p.getRadius() * cos(p.getGamma()/2);
	minSensedY = py - p.getRadius() ;
	maxSensedY = py;
	normalize(maxSensedY, 0);
	normalize(maxSensedX,maxValueX);
	normalize(minSensedX,0);
	//count how many sensed cells are occupied by an obstacles
	for(int i=minSensedX + ; i< maxSensedX; i++){
	    for (int j = minSensedY; j< maxSensedY; j++){
		/* If the intersection of the two segment, the one limited by the robot's position and the considered cell and the one limitating the circular sector, is false then 
		* the cell is inside the circular sector */
		if (intersect(i,j,minSensedX,minSensedY,p) == false || intersect(i,j,maxSensedX,minSensedY,p) == false)
		    sensedArea++;
		//Hp: free cells are zero value
		if(map.map2D[i].at(j) > 0){
		    occupiedArea++;
		}
	    }
	}
    }else if ( orientation == 'S'){
	minSensedX = px - p.getRadius() * cos(p.getGamma()/2);
	maxSensedX = px + p.getRadius() * cos(p.getGamma()/2);
	minSensedY = py ;
	maxSensedY = py + p.getRadius();
	normalize(maxSensedY, maxValueY);
	normalize(maxSensedX,maxValueX);
	normalize(minSensedX,0);
	for(int i=minSensedX + ; i< maxSensedX; i++){
	    for (int j = minSensedY; j< maxSensedY; j++){
		/* If the intersection of the two segment, the one limited by the robot's position and the considered cell and the one limitating the circular sector, is false then 
		* the cell is inside the circular sector */
		if (intersect(i,j,minSensedX,maxSensedY,p) == false || intersect(i,j,maxSensedX,maxSensedY,p) == false)
		    sensedArea++;
		//Hp: free cells are zero value
		if(map.map2D[i].at(j) > 0){
		    occupiedArea++;
		}
	    }
	}
    }else if ( orientation == 'E'){
	minSensedX = px;
	maxSensedX = px + p.getRadius();
	minSensedY = py - p.getRadius() * sin(p.getGamma()/2);
	maxSensedY = py + p.getRadius() * sin(p.getGamma()/2);
	normalize(maxSensedX,maxValueX);
	normalize(maxSensedY,0);
	normalize(minSensedY,maxValueY);
	for(int i=minSensedX + ; i< maxSensedX; i++){
	    for (int j = minSensedY; j< maxSensedY; j++){
		/* If the intersection of the two segment, the one limited by the robot's position and the considered cell and the one limitating the circular sector, is false then 
		* the cell is inside the circular sector */
		if (intersect(i,j,maxSensedX,minSensedY,p) == false || intersect(i,j,maxSensedX,maxSensedY,p) == false)
		    sensedArea++;
		//Hp: free cells are zero value
		if(map.map2D[i].at(j) > 0){
		    occupiedArea++;
		}
	    }
	}
    }else if (orientation == 'W'){
	maxSensedX = px;
	minSensedX = px - p.getRadius() ;
	minSensedY = py - p.getRadius() * sin(p.getGamma()/2);
	maxSensedY = py + p.getRadius() * sin(p.getGamma()/2);
	for(int i=minSensedX + ; i< maxSensedX; i++){
	    for (int j = minSensedY; j< maxSensedY; j++){
		/* If the intersection of the two segment, the one limited by the robot's position and the considered cell and the one limitating the circular sector, is false then 
		* the cell is inside the circular sector */
		if (intersect(i,j,minSensedX,minSensedY,p) == false || intersect(i,j,minSensedX,maxSensedY,p) == false)
		    sensedArea++;
		//Hp: free cells are zero value
		if(map.map2D[i].at(j) > 0){
		    occupiedArea++;
		}
	    }
	}
    }
    
    unExploredMap = sensedArea - occupiedArea;
    return unExploredMap;
}

void InformationGainCriterion::normalize (int position, int number)
{
    if(number == 0){
	 if(position <= 0){
	position = 0;
	}
    }else{
	if(position >= number){
	    position = number;
	}
    }

}

bool InformationGainCriterion::intersect(int p1x, int p1y, int p2x, int p2y, Pose p)
{
    int m1, m2;
    int num1, num2, den1, den2;
    
    num1 = p.getY() - p1y;
    den1 = p.getX() - p1x;
    m1 = num1 / den1;
    num2 = p.getY() - p2y;
    den2 = p.getX() - p2x;
    m2 = num2 / den2;
    //if angular coefficient are identical it means that two segments are parallel
    return (m1 == m2);
}



