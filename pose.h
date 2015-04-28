#ifndef POSE_H
#define POSE_H




class Pose 
{
public:
  Pose();
  Pose(int aX, int aY, int orientation, int range, double FOV);
  virtual ~Pose();
  double getDistance( Pose &pose);   
  int getX();
  int getY();
  int getOrientation();
  int getRange();
  double getFOV();
  bool isEqual(Pose &p);
  int getInformationGain();
  void setInformationGain(int value);

  
  
protected:
  int aX, aY;		// x and y coordinates of the cell
  int orientation;	// orientation theta of th robot
  int range;		// radius of the sensing operation 
  double FOV;		// central angle of the sensing operation
  int informationGain;
  
  
};


#endif