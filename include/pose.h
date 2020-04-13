#ifndef POSE_H
#define POSE_H

#include <utility>


class Pose
{
public:

  /**
   * Default Constructor
   */
  Pose();

  /**
   * Create a Pose object.
   * 
   * @param aX: the current x-coord of the robot
   * @param aY: the current y-coord of the robot
   * @param orientation: the orientation of the robot
   * @param range: the sensor range
   * @param FOV: the sensor FOV
   */
  Pose(long aX, long aY, int orientation, int range, double FOV);

  /**
   * Destructor
   */
  virtual ~Pose();

  /**
   * Get euclidean distance from a pose to the current one.
   * 
   * @param pose: the reference pose to calculate distance from
   * @return the euclidean distance
   */
  double getDistance( Pose &pose);

  /**
   * Return the current x-coord of the robot
   *
   *  @return the current x-coord of the robot
   */
  long getX();

  /**
   * Return the current y-coord of the robot
   * 
   * @return the current y-coord of the robot
   */
  long getY();

  /**
   * Return the current orientation of the robot
   * 
   * @return the current orientation of the robot
   */
  int getOrientation();

  /**
   * Return the current sensor range of the robot
   * 
   * @return the current sensor range of the robot
   */
  int getRange();

  /**
   * Return the current sensor FOV of the robot
   * 
   * @return the current sensor FOV of the robot
   */
  double getFOV();

  /**
   * Return the get informationGain value associated with the current pose
   * 
   * @return the informationGain value associated with the current pose
   */
  int getInformationGain();

  /**
   * Set the infoGain value for the current robot pose
   * 
   * @param value: the infoGain to assign to the current robot pose
   */
  void setInformationGain(int value);

  /**
   * Check if two poses are the same
   * 
   * @param p: the pose to check against the current one
   * @return true if the two poses are the same, false otherwise
   */
  bool isEqual(Pose &p);

  /**
   * Check if two poses are the same
   * 
   * @param p: the pose to check against the current one
   * @return true if the two poses are the same, false otherwise
   */
  bool operator==(const Pose &p);

  /**
   * Set the scanning angles for the curren pose
   * 
   * @param angles: the minimum and maximum angles required to cover the free area from the current pose
   */
  void setScanAngles(std::pair<double,double> angles);

  /**
   * Get the scanning angles associated with the current pose
   * 
   * @return the pair of scanning angles for the current pose
   */
  std::pair<double, double> getScanAngles();

  /**
   * Update the current pose using new values
   * 
   * @param position: a pair containing x and y coord of the robot
   * @param orientation: the orientation of the robot
   * @param range: the sensor range
   * @param FOV: the sensor FOV
   */
  void updateFromData(std::pair<long, long> position, float orientation, int range, double FOV);


protected:
  long aX, aY;		// x and y coordinates of the cell
  int orientation;	// orientation theta of th robot
  int range;		// radius of the sensing operation
  double FOV;		// central angle of the sensing operation
  int informationGain;
  std::pair<double,double> scanAngles;


};


#endif
