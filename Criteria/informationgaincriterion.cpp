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

double InformationGainCriterion::evaluate(Pose &p, Map &map)
{
    int px = p.getX();
    int py = p.getY();
    float resolution = map.getResolution();
    //Get the orientation
    int orientation = p.getOrientation();
    int minSensedX, maxSensedX;
    int minSensedY,maxSensedY;
    int *intersection;    
    vector<vector<int>> map2D = map.getMap2D;
    int maxValueY = map2D.size();
    int maxValueX = map2D[0].size();
    
    //area contained in the sensor cone
    int sensedArea;
    //area occupied by obstacles in the sensor cone
    int occupiedArea;
    //effective information gain
    int unExploredMap;
    
    /* Approximate way to calculate the sensed area inside the circular sector: calculate the area of the circular sector and 
     * then divide it by the map resolution to get the number of cell in it
    
    sensedArea = (int) 1/2 * pow(p.getRadius(),2) * p.getGamma();
    sensedArea = (int) sensedArea / map.map2D.getResolution();
    */
    
    //calcuate the sensed map based on the robot orientation
    if( orientation == 90 ){
	minSensedX = px - p.getR() * cos(p.getPhi()/2);
	maxSensedX = px + p.getR() * cos(p.getPhi()/2);
	minSensedY = py - p.getR() ;
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
		if(map2D[i].at(j) > 0){
		    occupiedArea++;
		}
	    }
	}
    }else if ( orientation == 270){
	minSensedX = px - p.getR() * cos(p.getPhi()/2);
	maxSensedX = px + p.getR() * cos(p.getPhi()/2);
	minSensedY = py ;
	maxSensedY = py + p.getR();
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
		if(map2D[i].at(j) > 0){
		    occupiedArea++;
		}
	    }
	}
    }else if ( orientation == 0){
	minSensedX = px;
	maxSensedX = px + p.getR();
	minSensedY = py - p.getR() * sin(p.getPhi()/2);
	maxSensedY = py + p.getR() * sin(p.getPhi()/2);
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
		if(map2D[i].at(j) > 0){
		    occupiedArea++;
		}
	    }
	}
    }else if (orientation == 180){
	maxSensedX = px;
	minSensedX = px - p.getR() ;
	minSensedY = py - p.getR() * sin(p.getPhi()/2);
	maxSensedY = py + p.getR() * sin(p.getPhi()/2);
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
		if(map2D[i].at(j) > 0){
		    occupiedArea++;
		}
	    }
	}
    }
    
    unExploredMap = sensedArea - occupiedArea;
    insertEvaluation(p,unExploredMap);
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

int InformationGainCriterion::intersect(int p1x, int p1y, int p2x, int p2y, Pose p)
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
     
   
    //First segment
    mNum1 = py - p1y;
    mDen1 = px - p1x;
    m1 = mNum1 / mDen1;
    qNum1 = px*p1y - p1x*py;
    qDen1 = px - p1x;
    q1 = qNum1 / qDen1;
    
    
    

   
    //Second segment: the radius at the end of the circular sector
    mNum2 = p.getY() - p2y;
    mDen2 = p.getX() - p2x;
    m2 = mNum2 / mDen2;
    qNum2 = px*p2y - p2x*py;
    qDen2 = px - p2x;
    q2 = qNum2 / qDen2;
    
    intersectX = (q2 - q1) / ( m1 - m2);
    intersectY = m1 * intersectX + q1;
    
    */
    
        //Vertical or horizontal segment from the considered cell 
   if(p.getOrientation == 90 || p.getOrientation == 270){
	intersectX = p1x;
	intersectY =py + (intersectX - px)*(p2y - py) / (p2x - px);     
    } else if (p.getOrientation == 0 || p.getOrientation == 180){
	intersectY = p1y;
	intersectX = p2x + (intersectY - py) * (p2x -px) / (p2y - py);
    }

    result[1] = (int) intersectX;
    result[2] = (int) intersectY;
    
    
    //return the coordinations of the interesection point
    return (result);
}



