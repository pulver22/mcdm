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

double RFIDCriterion::evaluate(Pose &p, dummy::Map *map, RFID_tools *rfid_tools, double *batteryTime) 
{
  float px = p.getX();
  float py = p.getY();
  // float resolution = map.getResolution();
  // Get the orientation
  float orientation = p.getOrientation();
  int range = p.getRange();
  double angle = p.getFOV();
  
  // 1) method using the ellipse map leading to results identical to informationGain
  // NOTE: ABANDONED
  // double unExploredMap = 0.0;
  // NewRay ray;
  //Map *map2 = &map;
  //   unExploredMap=(double)ray.getInformationGain(map,px,py,orientation,angle,range);
  // //  unExploredMap = map->getRFIDReading(px, py, orientation, angle, range); // TODO: re-enable when method add
  //   Criterion::insertEvaluation(p, unExploredMap);

  // 2) method using the belief maps
  // We sum all the pixels intensity (belief) for all the belief maps available
  // in a given area 5x5 around the robot
  this->RFIDInfoGain = 0.0;
  for (int tag_id = 0; tag_id < 10; tag_id++){
    this->tmp_belief = rfid_tools->rm->getTotalWeight(px, py, orientation, 5, 5, tag_id);
    if (isnan(tmp_belief)) this->tmp_belief = 0.0;  // belief outside corridors (into obstacles) is nan
    this->RFIDInfoGain += this->tmp_belief;
  }

  // 3) method using the total received power from all the tags in a given position 
  // double accumulated_received_power = 0.0;
  // for (int tag_id = 0; tag_id < 10; tag_id++){
  //   accumulated_received_power += rfid_tools->rm->received_power_friis(rfid_tools->tags_coord[tag_id].first, rfid_tools->tags_coord[tag_id].second, rfid_tools->freq, rfid_tools->txtPower);
  // }

  Criterion::insertEvaluation(p, this->RFIDInfoGain);
  return this->RFIDInfoGain;
}
