//
// Created by pulver on 29/07/2019.
//
#ifndef RFIDCRITERION_H
#define RFIDCRITERION_H

#include "criterion.h"
#include "map.h"
#include "pose.h"
#include <vector>

class RFIDCriterion : public Criterion {
public:
  RFIDCriterion(double weight);
  virtual ~RFIDCriterion();
  double evaluate(Pose &p, dummy::Map *map, RadarModel *rm, double *batteryTime);

private:
  void normalize(long minSensedX, int number);
  int *intersect(int p1x, int p1y, int p2x, int p2y, Pose &p);
};

#endif // RFIDCRITERION_H
