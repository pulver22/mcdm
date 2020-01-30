//
// Created by pulver on 29/07/2019.
//

#include "Criteria/RFIDCriterion.h"
#include "Criteria/criteriaName.h"
#include "newray.h"
#include <math.h>
using namespace dummy;

RFIDCriterion::RFIDCriterion(double weight)
    : Criterion(RFID_READING, weight, false) {
      // minValue = 0.0;
    }

RFIDCriterion::~RFIDCriterion() {}

double RFIDCriterion::evaluate(Pose &p, dummy::Map *map, RadarModel *rm) {

  float px = p.getX();
  float py = p.getY();
  // float resolution = map.getResolution();
  // Get the orientation
  float orientation = p.getOrientation();
  int range = p.getRange();
  double angle = p.getFOV();
  double unExploredMap = 0.0;
  NewRay ray;
  //Map *map2 = &map;
//   unExploredMap=(double)ray.getInformationGain(map,px,py,orientation,angle,range);
// //  unExploredMap = map->getRFIDReading(px, py, orientation, angle, range); // TODO: re-enable when method add
//   Criterion::insertEvaluation(p, unExploredMap);

  // NOTE: new method using the belief maps
  double RFIDInfoGain = 0.0;
  double tmp_belief = 0.0;
  for (int tag_id = 0; tag_id < 10; tag_id++){
    tmp_belief = rm->getTotalWeight(px, py, orientation, 5, 5, tag_id);
    if (isnan(tmp_belief)) tmp_belief = 0.0;  // belief outside corridors (into obstacles) is nan
    RFIDInfoGain += tmp_belief;
  }
  Criterion::insertEvaluation(p, RFIDInfoGain);
  return unExploredMap;
}
