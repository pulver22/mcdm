#ifndef POSE_H
#define POSE_H


  
class Pose 
{
public:
  Pose();
  Pose(int aX, int aY, double aTheta, int r, double phi);
  virtual ~Pose();
  double getDistance( Pose &pose);   
  int getX();
  int getY();
  double getOrientation();
  int getR();
  double getPhi();
  bool isEqual(Pose &p);
  
  
protected:
  int aX, aY;		// x and y coordinates of the cell
  double aTheta;	// orientation theta of th robot
  int r;		// radius of the sensing operation 
  double phi;		// central angle of the sensing operation
  
  
};
 
#endif