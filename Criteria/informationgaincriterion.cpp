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
    int *intersection;    
    int maxValueY = map.map2D.size();
    int maxValueX = map.map2D[0].size();
    
    //area contained in the sensor cone
    int sensedArea;
    //area occupied by obstacles in the sensor cone
    int occupiedArea;
    //effective information gain
    int unExploredMap;
    
    /* Approximate way to calculate the sensed area inside the circular sector
    //calculate the area of the circular sector and then divide it by the map resolution to get the number of cell in it
    sensedArea = (int) 1/2 * pow(p.getRadius(),2) * p.getGamma();
    sensedArea = (int) sensedArea / map.map2D.getResolution();
    */
    
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
	for(int i=minSensedY + ; i< maxSensedY; i++){
	    for (int j = minSensedX; j< maxSensedX; j++){
		/* If the intersection of the two segment, the one limited by the robot's position and the considered cell and the one limitating the circular sector, is false then 
		* the cell is inside the circular sector */
		if( j < px ){
		    intersection = intersect(i,j,minSensedX,minSensedY,p);
		    if ( i > intersection[1]){
			continue;
		    }
		}else{
		    intersection = intersect(i,j,maxSensedX,minSensedY,p);
		    if (i > intersection[1]){
		    continue;
		    }
		}
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
	for(int i=minSensedY + ; i< maxSensedY; i++){
	    for (int j = minSensedX; j< maxSensedX; j++){
		/* If the intersection of the two segment, the one limited by the robot's position and the considered cell and the one limitating the circular sector, is false then 
		* the cell is inside the circular sector */
		if( j < px ){
		    intersection = intersect(i,j,minSensedX,minSensedY,p);
		    if ( i < intersection[1]){
			continue;
		    }
		}else{
		    intersection = intersect(i,j,maxSensedX,minSensedY,p);
		    if (i < intersection[1]){
		    continue;
		    }
		}
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
	for(int i=minSensedY + ; i< maxSensedY; i++){
	    for (int j = minSensedX; j< maxSensedX; j++){
		/* If the intersection of the two segment, the one limited by the robot's position and the considered cell and the one limitating the circular sector, is false then 
		* the cell is inside the circular sector */
		if( i < py ){
		    intersection = intersect(i,j,maxSensedX,minSensedY,p);
		    if ( i < intersection[1]){
			continue;
		    }
		}else{
		    intersection = intersect(i,j,maxSensedX,maxSensedY,p);
		    if (i < intersection[1]){
		    continue;
		    }
		}
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
	for(int i=minSensedY + ; i< maxSensedY; i++){
	    for (int j = minSensedX; j< maxSensedX; j++){
		/* If the intersection of the two segment, the one limited by the robot's position and the considered cell and the one limitating the circular sector, is false then 
		* the cell is inside the circular sector */
		if( i < py ){
		    intersection = intersect(i,j,minSensedX,minSensedY,p);
		    if ( i < intersection[1]){
			continue;
		    }
		}else{
		    intersection = intersect(i,j,minSensedX,maxSensedY,p);
		    if (i < intersection[1]){
		    continue;
		    }
		}
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

int * InformationGainCriterion::intersect(int p1x, int p1y, int p2x, int p2y, Pose p)
{
    float m1, m2, q1, q2;
    float mNum1, mNum2, mDen1, mDen2;
    float qNum1, qNum2, qDen1, qDen2;
    float py = p.getY();
    float px = p.getX();
    float intersectX, intersectY;
    int result[2];
    /*
     *		x = (q' - q ) / (m - m') -> intersection point
     * 
     * 		m = (y2 - y1) / (x2 - x1)
     * 
     *		q = (x2*y1 - x1*y2) / (x2 - x1)
     */
   
    //Forst segment
    mNum1 = py - p1y;
    mDen1 = px - p1x;
    m1 = mNum1 / mDen1;
    qNum1 = px*p1y - p1x*py;
    qDen1 = px - p1x;
    q1 = qNum1 / qDen1;
    
    //Second segment
    mNum2 = p.getY() - p2y;
    mDen2 = p.getX() - p2x;
    m2 = mNum2 / mDen2;
    qNum2 = px*p2y - p2x*py;
    qDen2 = px - p2x;
    q2 = qNum2 / qDen2;
    
    intersectX = (q2 - q1) / ( m1 - m2);
    intersectY = m1 * intersectX + q1;
    
    result[1] = intersectX;
    result[2] = intersectY;
    
    //if angular coefficient are identical it means that two segments are parallel
    return (result);
}



