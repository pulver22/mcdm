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
    : Criterion(RFID_READING, weight, false) {
  // minValue = 0.0;
}

RFIDCriterion::~RFIDCriterion() {}

double RFIDCriterion::evaluate(Pose &p, dummy::Map *map, RFID_tools *rfid_tools,
                               double *batteryTime) {

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
  float px = p.getX();
  float py = p.getY();
  // float resolution = map.getResolution();
  // Get the orientation
  float orientation = p.getOrientation();
  int range = p.getRange();
  double angle = p.getFOV();

  double RFIDInfoGain = 0.0;
  double tmp_belief = 0.0;
  for (int tag_id = 0; tag_id < 10; tag_id++) {
    tmp_belief =
        rfid_tools->rm.getTotalWeight(px, py, orientation, 5, 5, tag_id);
    if (isnan(tmp_belief))
      tmp_belief = 0.0; // belief outside corridors (into obstacles) is nan
    RFIDInfoGain += tmp_belief;
  }

  return RFIDInfoGain;
}

double RFIDCriterion::evaluateEntropyOverBelief(Pose &p, dummy::Map *map,
                                                RFID_tools *rfid_tools) {
  float px = p.getX();
  float py = p.getY();

  Position rel_point = Position(px, py);
  double likelihood = 0.0;
  double entropy_cell = 0.0;
  GridMap::Matrix grid;
  int buffer_size = 3;

  // Iterate on a submap [buffer_size * buffer_size]
  // First check the submap is not outside map borders
  if (px > buffer_size and
      px <= rfid_tools->rm.getBeliefMaps().getLength().x() - buffer_size) {
    if (py > buffer_size and
        py <= rfid_tools->rm.getBeliefMaps().getLength().y() - buffer_size) {

      // Create indexes at the beginning and end of submap
      grid_map::Index submapStartIndex(px - buffer_size, py - buffer_size);
      grid_map::Index submapBufferSize(buffer_size, buffer_size);

      // Iterate over the submap
      for (grid_map::SubmapIterator sub_iterator(rfid_tools->rm.getBeliefMaps(),
                                                 submapStartIndex,
                                                 submapBufferSize);
           !sub_iterator.isPastEnd(); ++sub_iterator) {
        // Create index for the submap
        grid_map::Index sub_index(*sub_iterator);

        // Get the entropy for the current cell for every map
        for (int tag_id = 0; tag_id < rfid_tools->tags_coord.size(); tag_id++) {
          string layerName = rfid_tools->rm.getTagLayerName(tag_id);
          grid = rfid_tools->rm.getBeliefMaps()[layerName];
          likelihood = grid(sub_index(0), sub_index(1));
          // likelihood =
          // rfid_tools->rm.getBeliefMaps().atPosition(layerName,rel_point);
          entropy_cell += -likelihood * log2(likelihood) -
                          (1 - likelihood) * log2(1 - likelihood);
        }
      }
    }
  }

  return entropy_cell;
}