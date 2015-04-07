#ifndef POSE_H
#define POSE_H

namespace Data{

  
class Pose
{
public:
  Pose();
  Pose(int aX, int aY, double aTheta, int r, double phi);
  virtual ~Pose();
  
  double getDistance(const Pose &pose); 
  
  int getX();
  int getY();
  double getOrientation();
  int getR();
  double getPhi();
  
  
private:
  int aX, aY;		// x and y coordinates of the cell
  double aTheta;	// orientation theta of th robot
  int r;		// radius of the sensing operation 
  double phi;		// central angle of the sensing operation
  
  
};
 
}