#include "pose.h"
#include <cmath>

using namespace std;
Pose::Pose():
  aX(0),
  aY(0),
  aTheta(0),
  r(0),
  phi(0) 
   {
}

Pose::Pose(int aX, int aY, double aTheta, int r, double phi):
    aX(aX), 
    aY(aY), 
    aTheta(aTheta), 
    r(r),
    phi(phi)
{
}


Pose::~Pose()
{
  
}

double Pose::getDistance( Pose& pose)
{
  return sqrt((aX - pose.getX())*(aX - pose.getX()) + (aY - pose.getY())*(aY - pose.getY()));
}

int Pose::getX()
{
  return aX;
}

int Pose::getY()
{
  return aY;
}

double Pose::getOrientation()
{
  return aTheta;
}

int Pose::getR()
{
  return r;
}

double Pose::getPhi()
{
  return phi;
}

bool Pose::isEqual(Pose& p)
{
    if(aX == p.getX() & aY == p.getY() & aTheta == p.getOrientation() & phi == p.getPhi() & r == p.getR()){
	return true;
    } else return false;
    
}


void Pose::operator=(Pose& other)
{
    aX = other.getX();
    aY = other.getY();
    r = other.getR();
    aTheta = other.getOrientation();
    phi = other.getPhi();
}





