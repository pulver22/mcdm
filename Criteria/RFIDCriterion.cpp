//
// Created by pulver on 29/07/2019.
//

#include "Criteria/RFIDCriterion.h"
#include "Criteria/criteriaName.h"
#include "Eigen/Eigen"
#include "newray.h"
#include <math.h>
using namespace dummy;
using namespace grid_map;

RFIDCriterion::RFIDCriterion(double weight)
    : Criterion(RFID_READING, weight, true) {
  // minValue = 0.0;
}

RFIDCriterion::~RFIDCriterion() {}

double RFIDCriterion::evaluate(Pose &p, dummy::Map *map, RFID_tools *rfid_tools,
                               double *batteryTime) {

  // 1) Use a uniform ellipse (DEPRECATED: identical to infoGain)
  // this->RFIDInfoGain = evaluateUniformEllipse(p, map);

  // 2) Sum all the likelihood around the cells
  // this->RFIDInfoGain = evaluateSumOverBelief(p, map, rfid_tools);

  // 3) Calculate entropy around the cell
  this->RFIDInfoGain = evaluateEntropyOverBelief(p, map, rfid_tools);

  Criterion::insertEvaluation(p, this->RFIDInfoGain);
  return this->RFIDInfoGain;
}

double RFIDCriterion::evaluateUniformEllipse(Pose &p, dummy::Map *map) {
  float px = p.getX();
  float py = p.getY();
  // float resolution = map.getResolution();
  // Get the orientation
  float orientation = p.getOrientation();
  int range = p.getRange();
  double angle = p.getFOV();

  double unExploredMap = 0.0;
  NewRay ray;
  return (double)ray.getInformationGain(map, px, py, orientation, angle, range);
}

double RFIDCriterion::evaluateSumOverBelief(Pose &p, dummy::Map *map,
                                            RFID_tools *rfid_tools) {
  double RFIDInfoGain = 0.0;
  double tmp_belief = 0.0;
  int buffer_size = 2;

  for (int tag_id = 0; tag_id < 10; tag_id++) {
    tmp_belief =
        rfid_tools->rm.getTotalWeight(p.getX(), p.getY(), p.getOrientation(),
                                      buffer_size, buffer_size, tag_id);
    if (isnan(tmp_belief))
      tmp_belief = 0.0; // belief outside corridors (into obstacles) is nan
    RFIDInfoGain += tmp_belief;
  }

  return RFIDInfoGain;
}

double RFIDCriterion::evaluateEntropyOverBelief(Pose &p, dummy::Map *map,
                                                RFID_tools *rfid_tools) {
  float RFIDInfoGain = 0.0;
  double entropy_cell = 0.0;
  int buffer_size = 2;

  for (int tag_id = 0; tag_id < 10; tag_id++) {
    entropy_cell =
        rfid_tools->rm.getTotalEntropy(p.getX(), p.getY(), p.getOrientation(),
                                       buffer_size, buffer_size, tag_id);
    RFIDInfoGain += entropy_cell;
  }

  return RFIDInfoGain;
}