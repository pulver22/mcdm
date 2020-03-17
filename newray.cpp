#include <vector>
#include "math.h"
#include "map.h"

#include "newray.h"

# define PI           3.14159265358979323846  /* pi */



NewRay::NewRay()
{
}

NewRay::~NewRay()
{
}

int NewRay::isCandidate(const dummy::Map *map, long i,long  j)
{
  int candidate = 0;
  long r = i;
  long s = j;
  long minR = r - 1, maxR = r + 1, minS = s -1, maxS = s + 1;
  if(minR < 0) minR = 0;
  if(minS < 0) minS = 0;
  if(maxR > map->getPathPlanningNumRows()) maxR = map->getPathPlanningNumRows();
  if(maxS > map->getPathPlanningNumCols()) maxS = map->getPathPlanningNumCols();


  for(r = minR; r <= maxR; ++r)
  {
    for(s = minS; s <= maxS; ++s)
    {
      if (map->getPathPlanningGridValue(r, s) == 0) candidate = 1;
    }
  }
  return candidate;

}

int NewRay::isCandidate2(const dummy::Map *map, long i, long j)
{
  int candidate = 0;
  long r = i;
  long s = j;
  long minR = r - 1, maxR = r + 1, minS = s -1, maxS = s + 1;
  if(minR < 0) minR = 0;
  if(minS < 0) minS = 0;
  if(maxR > map->getPathPlanningNumRows()) maxR = map->getPathPlanningNumRows();
  if(maxS > map->getPathPlanningNumCols()) maxS = map->getPathPlanningNumCols();


  for(r = minR; r <= maxR; ++r)
  {
    for(s = minS; s <= maxS; ++s)
    {
      for(int rg = r*gridToPathGridScale; rg < r*gridToPathGridScale + gridToPathGridScale; ++rg)
      {
        for(int sg = s*gridToPathGridScale; sg < s*gridToPathGridScale + gridToPathGridScale; ++sg)
        {
          if (map->getGridValue(rg, sg) == 0) candidate = 1;
        }
      }
    }
  }
  return candidate;

}


void NewRay::findCandidatePositions(dummy::Map *map, long posX, long posY, int orientation, double FOV, int range)
{
  NewRay::numGridRows = map->getNumGridRows();
  NewRay::numPathPlanningGridCols = map->getPathPlanningNumCols();
  NewRay::numPathPlanningGridRows = map->getPathPlanningNumRows();
  NewRay::gridToPathGridScale = map->getGridToPathGridScale();

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
  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
  long minI = posX - range;
  long maxI = posX + range;
  long minJ = posY - range;
  long maxJ = posY + range;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getPathPlanningNumRows()) maxI = map->getPathPlanningNumRows();
  if(maxJ > map->getPathPlanningNumCols()) maxJ = map->getPathPlanningNumCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {

      double distance = sqrt((i - posX)*(i - posX) + (j - posY)*(j - posY));
      //cout << map->getGridValue(i, j) << " : " << distance << " : " <<range << endl;

      //if a cell is a candidate one and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getPathPlanningGridValue(i, j) == 2 && distance <= range)
      {

        if(NewRay::isCandidate(map, i, j) == 1)
        {

          double curX = posX;		//starting position of the ray
          double curY = posY;
          double robotX = posX;		//position of the robot
          double robotY = posY;

          double convertedI = NewRay::convertPointPP(i);
          double convertedRX = NewRay::convertPointPP(robotX);

          double slope = atan2(NewRay::convertPointPP(i) - NewRay::convertPointPP(robotX), j - robotY);	//calculate the slope of the ray with atan2

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
              if(curX < 0 || curX > map->getPathPlanningNumRows() || curY < 0 || curY > map->getPathPlanningNumCols()) hit = 1;

              if(map->getPathPlanningGridValue((long)curX, (long)curY) == 1)
              {
                hit = 1;		//hit set to 1 if an obstacle is found
                //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
              }


              if((long)curX == i && (long)curY == j)	//if the free cell is reached, save it as edge point and stop the ray.
              {
                std::pair<long,long> temp = std::make_pair(i, j);
                NewRay::edgePoints.push_back(temp);
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

void NewRay::findCandidatePositions2(dummy::Map *map, long posX, long posY, int orientation, double FOV, int range)
{
  NewRay::numGridRows = map->getNumGridRows();
  NewRay::numPathPlanningGridCols = map->getPathPlanningNumCols();
  NewRay::numPathPlanningGridRows = map->getPathPlanningNumRows();
  NewRay::gridToPathGridScale = map->getGridToPathGridScale();

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
  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
  long minI = posX - range;
  long maxI = posX + range;
  long minJ = posY - range;
  long maxJ = posY + range;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getPathPlanningNumRows()) maxI = map->getPathPlanningNumRows();
  if(maxJ > map->getPathPlanningNumCols()) maxJ = map->getPathPlanningNumCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {

      double distance = sqrt((i - posX)*(i - posX) + (j - posY)*(j - posY));
      //cout << map->getGridValue(i, j) << " : " << distance << " : " <<range << endl;

      //if a cell is a candidate one and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getPathPlanningGridValue(i, j) == 2 && distance <= range)
      {

        if(NewRay::isCandidate2(map, i, j) == 1)
        {

          double curX = posX;		//starting position of the ray
          double curY = posY;
          double robotX = posX;		//position of the robot
          double robotY = posY;

          double convertedI = NewRay::convertPointPP(i);
          double convertedRX = NewRay::convertPointPP(robotX);

          double slope = atan2(NewRay::convertPointPP(i) - NewRay::convertPointPP(robotX), j - robotY);	//calculate the slope of the ray with atan2

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
              if(curX < 0 || curX > map->getPathPlanningNumRows() || curY < 0 || curY > map->getPathPlanningNumCols()) hit = 1;

              if(map->getPathPlanningGridValue((long)curX, (long)curY) == 1)
              {
                hit = 1;		//hit set to 1 if an obstacle is found
                //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
              }


              if((long)curX == i && (long)curY == j)	//if the free cell is reached, save it as edge point and stop the ray.
              {
                std::pair<long,long> temp = std::make_pair(i, j);
                NewRay::edgePoints.push_back(temp);
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

vector< std::pair<long,long> > NewRay::getCandidatePositions()
{
  return NewRay::edgePoints;
}

void NewRay::emptyCandidatePositions()
{
  NewRay::edgePoints.clear();
}


std::pair<double,double> NewRay::getSensingTime(const dummy::Map *map, long posX,long posY, int orientation, double FOV, int range)
{

  NewRay::numGridRows = map->getNumGridRows();
  setGridToPathGridScale(map->getGridToPathGridScale());


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

  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
  long minI = posX*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxI = posX*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
  long minJ = posY*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxJ = posY*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {

      double distance = sqrt((i - posX*gridToPathGridScale)*(i - posX*gridToPathGridScale) + (j - posY*gridToPathGridScale)*(j - posY*gridToPathGridScale));

      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range*gridToPathGridScale)
      {
        double curX = posX*gridToPathGridScale + gridToPathGridScale/2;		//starting position of the ray
        double curY = posY*gridToPathGridScale + gridToPathGridScale/2;
        double robotX = posX*gridToPathGridScale + gridToPathGridScale/2;		//position of the robot
        double robotY = posY*gridToPathGridScale + gridToPathGridScale/2;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope = atan2(NewRay::convertPoint(i) - NewRay::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2

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

            if(map->getGridValue((long)curX, (long)curY) == 1)
            {
              hit = 1;		//hit set to 1 if an obstacle is found
              //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
            }

            if((long)curX == i && (long)curY == j)	//free cell reached, check if the slope is a candidate for first or last ray
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

  /*
  if(phiFound == 0) return -1;		//return -1 if no free cells can be scanned
  else 					//return the correct FOV (ALWAYS CENTERED ON THE ORIENTATION)
  {
    if(minPhi - startingPhi <= endingPhi - maxPhi) value = (endingPhi - startingPhi - 2*(minPhi - startingPhi));
  else value = (endingPhi - startingPhi - 2*(endingPhi - maxPhi));
  }

  //std::cout << "startingPhi " << startingPhi << " endingPhi " << endingPhi << " minPhi " << minPhi << " maxPhi " << maxPhi << std::endl;

  return value;
  */
  // return sensingTime;
  std::pair<double, double> angles;
  angles.first = minPhi;
  angles.second = maxPhi;
  return angles;
}

int NewRay::performSensingOperationEllipse(dummy::Map *map, long posX,
                                           long posY, int posOri,
                                           double firstAngle, double lastAngle,
                                           long a_pcell, long b_pcell,
                                           bool debug) {

  /*
  dummy::Map  map,
  long posX robot coordinate x position in planning cells, focal point
  long posY robot coordinate y position in planning cells, focal point
  int  posOri   robot orientation in degrees
  double firstAngle FOV starting angle in radians
  double lastAngle  FOV end angle in radians
  long a_pcell   #ellipse radius on the x-axis in planning cells
  long b_pcell   #ellipse radius on the y-axis in planning cells
  */

  if (debug) {
    printf("..................................................................."
           ". \n");
    printf("Input data: \n");
    printf("\t - Robot pose %lu, %lu (pCell) heading (%d) (degs) \n", posX,
           posY, posOri);
    printf("\t - FoV: %3.3f and %3.3f (rads) \n", firstAngle, lastAngle);
    printf("\t - Mayor radius a: %lu (pCell) \n", a_pcell);
    printf("\t - Minor radius b: %lu (pCell) \n", b_pcell);
  }
  NewRay::numGridRows = map->getNumGridRows();
  setGridToPathGridScale(map->getGridToPathGridScale());
  int counter = 0;

  // set the correct FOV orientation
  double orientation = posOri * PI / 180.0;

  double startingPhi = firstAngle;
  double endingPhi = lastAngle;
  int add2pi = 0;

  if (startingPhi <= 0) {
    add2pi = 1;
    startingPhi = 2 * PI + startingPhi;
    endingPhi = 2 * PI + endingPhi;
  }

  if (endingPhi > 2 * PI)
    add2pi = 1;

  // data check
  if ((a_pcell == 0) || (b_pcell == 0)) {
    std::cout << "INVALID ELLIPSE RADIUS!!!" << std::endl;
    return 0;
  }

  if (a_pcell < b_pcell) {
    long tmp;
    tmp = a_pcell;
    a_pcell = b_pcell;
    b_pcell = tmp;
    std::cout << "minor radius smaller than mayor one!" << std::endl;
  }

  // get some relevant points of the ellipse in cell units
  long c_pcell = sqrt((a_pcell * a_pcell) - (b_pcell * b_pcell));

  double a_cell = a_pcell * gridToPathGridScale + gridToPathGridScale / 2;
  double b_cell = b_pcell * gridToPathGridScale + gridToPathGridScale / 2;
  double c_cell = c_pcell * gridToPathGridScale + gridToPathGridScale / 2;

  if (debug) {
    printf("\t - Centre to focus dist c: %lu (pCell) \n", c_pcell);
    printf("\n");
  }
  // first focal point of the ellipse: robot pose
  double x_f1_cell = posX * gridToPathGridScale + gridToPathGridScale / 2;
  double y_f1_cell = posY * gridToPathGridScale + gridToPathGridScale / 2;

  double robotX = x_f1_cell;
  double robotY = y_f1_cell;

  // center of the ellipse: to get ranges
  double x_0_cell = x_f1_cell + (c_cell * cos(orientation));
  double y_0_cell = y_f1_cell + (c_cell * sin(orientation));

  // second focal point of the ellipse: to get distances
  double x_f2_cell = x_0_cell + (c_cell * cos(orientation));
  double y_f2_cell = y_0_cell + (c_cell * sin(orientation));

  if (debug) {
    printf("Ellipse params in nav cell scale: \n");
    printf("\t - Mayor radius a: %3.1f (nCell) \n", a_cell);
    printf("\t - Minor radius b: %3.1f (nCell) \n", b_cell);
    printf("\t - Centre to focus dist c: %3.1f (nCell) \n", c_cell);
    printf("\n");
    printf("\t - Centre  %3.1f, %3.1f (nCell)\n", x_0_cell, y_0_cell);
    printf("\t - Focal point 1  %3.1f, %3.1f (nCell) (robot pose)\n", x_f1_cell,
           y_f1_cell);
    printf("\t - Focal point 2  %3.1f, %3.1f (nCell) (robot pose)\n", x_f2_cell,
           y_f2_cell);
    printf("\n");
  }
  // select the portion of map to be scanned
  // no matter the orientation of the ellipse, a is the mayor radius

  long minI = std::floor(x_0_cell - a_cell);
  long maxI = std::floor(x_0_cell + a_cell);
  long minJ = std::floor(y_0_cell - a_cell);
  long maxJ = std::floor(y_0_cell + a_cell);

  if (minI < 0)
    minI = 0;
  if (minJ < 0)
    minJ = 0;
  if (maxI > map->getNumGridRows())
    maxI = map->getNumGridRows();
  if (maxJ > map->getNumGridCols())
    maxJ = map->getNumGridCols();
  if (debug) {
    printf("Update bounding box in nav cell scale: \n");
    printf("\t - minI,minJ  %lu, %lu (nCell) \n", minI, minJ);
    printf("\t - maxI,maxJ  %lu, %lu (nCell) \n", maxI, maxJ);
    printf("\n");
  }

  // scan the cells in the selected portion of the map
  for (long i = minI; i <= maxI; ++i) {
    for (long j = minJ; j <= maxJ; ++j) {

      // in an ellipse, sum of distance to focal points is constant
      // double px = i
      // double py =

      double d1 = sqrt(pow(i - x_f1_cell, 2) + pow(j - y_f1_cell, 2));
      double d2 = sqrt(pow(i - x_f2_cell, 2) + pow(j - y_f2_cell, 2));
      bool isInside = (d1 + d2 <= (2 * a_cell));
      if (debug) {
        printf("Dists from  %lu, %lu (nCell) to Focal points \n", i, j);
        printf("\t to F1 == (%3.1f) (nCell) \n", d1);
        printf("\t to F2 == (%3.1f) (nCell) \n", d2);
        if (isInside) {
          printf("\t Inside! \n");
        } else {
          printf("\t Outside! \n");
        }
        printf("\n");
      }

      // if a cell is free and within range of the robot, generate the ray
      // connecting the robot cell and the free cell
      if (map->getGridValue(i, j) == 0 && isInside) {
        double curX = robotX; // starting position of the ray
        double curY = robotY;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope =
            atan2(convertedI - convertedRX,
                  j - robotY); // calculate the slope of the ray with atan2

        if (slope <= 0 && add2pi == 0)
          slope = slope + 2 * PI;
        if (add2pi == 1)
          slope = 2 * PI + slope; // needed in case of FOV spanning from
                                  // negative to positive angle values

        // std::cout << std::endl << "StartingPhi: " << startingPhi << "
        // EndingPhi: " << endingPhi <<std::endl;

        if (slope >= startingPhi &&
            slope <= endingPhi) // only cast the ray if it is inside the FOV of
                                // the robot
        {
          // raycounter++;
          // std::cout << "Inside loop, slope: " << slope  << " Cell: " << j <<
          // " " << i << std::endl;

          int hit = 0; // set to 1 when obstacle is hit by ray or when the cell
                       // is reached in order to stop the ray
          double u = 0;    // current position along the ray
          while (hit == 0) // scan the map along the ray until an ostacle is
                           // found or the considered cell is reached
          {

            // convert the position on the ray to cell coordinates to check the
            // grid
            curY = robotY + 0.5 + u * cos(slope);
            curX = robotX + 0.5 - u * sin(slope);

            // not needed, but left anyway
            if (curX < 0 || curX > map->getNumGridRows() || curY < 0 ||
                curY > map->getNumGridCols())
              hit = 1;

            if (map->getGridValue((long)curX, (long)curY) == 1) {
              hit = 1; // hit set to 1 if an obstacle is found
              // std::cout << "HIT! cell: " << j << " " << i << " Hit point: "
              // << curY << " " << curX << std::endl;
            }

            if ((long)curX == i &&
                (long)curY == j) // if the free cell is reached, set its value
                                 // to 2 and stop the ray
            {
              map->setGridValue(2, i, j);
              counter++;
              // std::cout << "Cell scanned: " << (int)curY << " " << (int)curX
              // << std::endl;
              hit = 1;
            }
            u += 0.2; // move forward along the ray
          }
        }
      }
    }
  }

  if (debug) {
    printf("..................................................................."
           ". \n\n\n");
  }
  return counter;
}


int NewRay::performSensingOperation(dummy::Map *map, long posX, long posY, int orientation, double FOV, int range, double firstAngle, double lastAngle)
{

  NewRay::numGridRows = map->getNumGridRows();
  setGridToPathGridScale(map->getGridToPathGridScale());
  int counter = 0;

  //set the correct FOV orientation
  double startingPhi = firstAngle; //orientation*PI/180 - FOV/2;
  double endingPhi = lastAngle; //orientation*PI/180 + FOV/2;
  int add2pi = 0;

  if(startingPhi <= 0)
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }

  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
  long minI = posX*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxI = posX*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
  long minJ = posY*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxJ = posY*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {

      double distance = sqrt((i - posX*gridToPathGridScale)*(i - posX*gridToPathGridScale) + (j - posY*gridToPathGridScale)*(j - posY*gridToPathGridScale));

      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range*gridToPathGridScale)
      {
        double curX = posX*gridToPathGridScale + gridToPathGridScale/2;		//starting position of the ray
        double curY = posY*gridToPathGridScale + gridToPathGridScale/2;
        double robotX = posX*gridToPathGridScale + gridToPathGridScale/2;		//position of the robot
        double robotY = posY*gridToPathGridScale + gridToPathGridScale/2;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope = atan2(NewRay::convertPoint(i) - NewRay::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2

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

            if(map->getGridValue((long)curX, (long)curY) == 1)
            {
              hit = 1;		//hit set to 1 if an obstacle is found
              //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
            }

            if((long)curX == i && (long)curY == j)	//if the free cell is reached, set its value to 2 and stop the ray
            {
              map->setGridValue(2, i, j);
              counter++;
              //std::cout << "Cell scanned: " << (int)curY << " " << (int)curX << std::endl;
              hit = 1;
            }
            u += 0.2;		//move forward along the ray
          }
        }
      }
    }
  }
  return counter;
}


long NewRay::convertPoint(long y)
{
  return (NewRay::numGridRows - 1 - y);
}


long NewRay::convertPointPP(long y)
{
  return (NewRay::numPathPlanningGridRows - 1 - y);
}


int NewRay::getInformationGain(const dummy::Map *map, long posX, long posY, int orientation, double FOV, int range)
{
  //int raycounter = 0;
  setGridToPathGridScale(map->getGridToPathGridScale());
  int counter = 0;	//count number of free cells that can be seen
  NewRay::numGridRows = map->getNumGridRows();

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

  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
  long minI = posX*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxI = posX*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
  long minJ = posY*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxJ = posY*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {
      double distance = sqrt((i - posX*gridToPathGridScale)*(i - posX*gridToPathGridScale) + (j - posY*gridToPathGridScale)*(j - posY*gridToPathGridScale));

      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range*gridToPathGridScale)
      {
        double curX = posX*gridToPathGridScale + gridToPathGridScale/2;		//starting position of the ray
        double curY = posY*gridToPathGridScale + gridToPathGridScale/2;
        double robotX = posX*gridToPathGridScale + gridToPathGridScale/2;		//position of the robot
        double robotY = posY*gridToPathGridScale + gridToPathGridScale/2;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope = atan2(NewRay::convertPoint(i) - NewRay::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2

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
            if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols())
            {
              hit = 1;
              //break;
            }

            if(map->getGridValue((long)curX, (long)curY) == 1)
            {
              hit = 1;		//hit set to 1 if an obstacle is found
              //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
            }

            if((long)curX == i && (long)curY == j)	//if the free cell is reached, increase counter and stop the ray.
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

  // return this->informationGain;
}


int NewRay::getRFIDGain(const dummy::Map *map, long posX, long posY, int orientation, double FOV, int range)
{
  //int raycounter = 0;
  setGridToPathGridScale(map->getGridToPathGridScale());
  int counter = 0;	//count number of free cells that can be seen
  NewRay::numGridRows = map->getNumGridRows();

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

  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
  long minI = posX*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxI = posX*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
  long minJ = posY*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxJ = posY*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {
      double distance = sqrt((i - posX*gridToPathGridScale)*(i - posX*gridToPathGridScale) + (j - posY*gridToPathGridScale)*(j - posY*gridToPathGridScale));

      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getRFIDGridValue(i, j) > 0 && distance <= range*gridToPathGridScale)
      {
        double curX = posX*gridToPathGridScale + gridToPathGridScale/2;		//starting position of the ray
        double curY = posY*gridToPathGridScale + gridToPathGridScale/2;
        double robotX = posX*gridToPathGridScale + gridToPathGridScale/2;		//position of the robot
        double robotY = posY*gridToPathGridScale + gridToPathGridScale/2;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope = atan2(NewRay::convertPoint(i) - NewRay::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2

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
            if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols())
            {
              hit = 1;
              //break;
            }

            if(map->getGridValue((long)curX, (long)curY) == 1)
            {
              hit = 1;		//hit set to 1 if an obstacle is found
              //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
            }

            if((long)curX == i && (long)curY == j)	//if the free cell is reached, increase counter and stop the ray.
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

  // return this->informationGain;
}


int NewRay::setGridToPathGridScale(int value)
{
  gridToPathGridScale = value;
}


void NewRay::performRFIDSensingOperation(dummy::Map *map, long posX, long posY, int orientation, double FOV, int range, double power, double firstAngle, double lastAngle)
{
  cout << "[" << posX << "," << posY << "]" << endl;
  NewRay::numGridRows = map->getNumGridRows();
  setGridToPathGridScale(map->getGridToPathGridScale());
  int counter = 0;

  //set the correct FOV orientation
  double startingPhi = firstAngle; //orientation*PI/180 - FOV/2;
  double endingPhi = lastAngle; //orientation*PI/180 + FOV/2;
  int add2pi = 0;

  if(startingPhi <= 0)
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }

  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
  long minI = posX*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxI = posX*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
  long minJ = posY*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
  long maxJ = posY*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {

      double distance = sqrt((i - posX*gridToPathGridScale)*(i - posX*gridToPathGridScale) + (j - posY*gridToPathGridScale)*(j - posY*gridToPathGridScale));

      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range*gridToPathGridScale)
      {
        double curX = posX*gridToPathGridScale + gridToPathGridScale/2;		//starting position of the ray
        double curY = posY*gridToPathGridScale + gridToPathGridScale/2;
        double robotX = posX*gridToPathGridScale + gridToPathGridScale/2;		//position of the robot
        double robotY = posY*gridToPathGridScale + gridToPathGridScale/2;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope = atan2(NewRay::convertPoint(i) - NewRay::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2

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

            if(map->getGridValue((long)curX, (long)curY) == 1)
            {
              hit = 1;		//hit set to 1 if an obstacle is found
              //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
            }

            if((long)curX == i && (long)curY == j)	//if the free cell is reached, set its value to 2 and stop the ray
            {
              map->setRFIDGridValue(50, i, j);
              counter++;
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
