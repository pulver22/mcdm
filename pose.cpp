#include "pose.h"
#include <cmath>

namespace Data{

Pose::Pose()
{
  aX(0);
  aY(0);
  aTheta(0);
  r(0);
  phi(0);
}

Pose::Pose(int aX, int aY, double aTheta, int r, int phi):
    aX(aX), aY(aY), aTheta(aTheta), r(r), phi(phi)
{
}

Pose::~Pose()
{
  
}

double Pose::getDistance(const Pose& pose)
{
  return std::sqrt((aX - pose.getX())*(aX - pose.getX()) + (aY - pose.getY())*(aY - pose.getY()));
}

int Pose::getX()
{
  return aX;
}

int Pose::getY()
{
  return aY;
}

double Pose::getTheta()
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




}