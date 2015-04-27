#ifndef POSE_H
#define POSE_H
#include <functional>


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
  bool operator==(const Pose &other) const;
  void operator=(Pose &other);
  
  
protected:
  int aX;
  int aY;		// x and y coordinates of the cell
  double aTheta;	// orientation theta of th robot
  int r;		// radius of the sensing operation 
  double phi;		// central angle of the sensing operation
  
  
};

/*
 * ATTENTION: need to be used if we want to use Pose type as key in the unordered_map
inline bool Pose::operator==(const Pose &other) const
{ return ( aX == other.aX
            && aY == other.aY
            && aTheta == other.aTheta
	    && r == other.r
	    && phi == other.phi);
}

namespace std{
    template <>
	struct hash<Pose>
	{
	inline std::size_t operator()(const Pose& k) const
	    {
		using std::size_t;
		using std::hash;

		// Compute individual hash values for first,
		// second and third and combine them using XOR
		// and bit shifting:

		return ((hash<int>()(k.aX)
			^ (hash<int>()(k.aY) << 1)) >> 1)
			^ ((hash<double>()(k.aTheta) << 1)
			^ (hash<int>()(k.r) <<1) >>1)
			^ (hash<double>()(k.phi)<<1);
	    }
    };
}
*/


 
#endif