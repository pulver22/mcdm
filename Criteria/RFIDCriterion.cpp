//
// Created by pulver on 29/07/2019.
//

#include "Criteria/RFIDCriterion.h"
#include "Criteria/criteriaName.h"
#include "newray.h"
#include <math.h>
using namespace dummy;

RFIDCriterion::RFIDCriterion(double weight)
    : Criterion(RFID_READING, weight, true) {
    }

RFIDCriterion::~RFIDCriterion() {}

double RFIDCriterion::evaluate(Pose &p, dummy::Map &map) {

  float px = p.getX();
  float py = p.getY();
  // float resolution = map.getResolution();
  // Get the orientation
  float orientation = p.getOrientation();
  int range = p.getRange();
  double angle = p.getFOV();
  double unExploredMap = 0;
  NewRay ray;
  //Map *map2 = &map;
  unExploredMap=(double)ray.getInformationGain(map,px,py,orientation,angle,range);
//  unExploredMap = map->getRFIDReading(px, py, orientation, angle, range); // TODO: re-enable when method add
  Criterion::insertEvaluation(p, unExploredMap);
  return unExploredMap;
}
