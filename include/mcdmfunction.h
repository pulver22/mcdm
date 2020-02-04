#ifndef MCDMFUNCTION_H
#define MCDMFUNCTION_H
#include "Criteria/criterion.h"
#include "Criteria/weightmatrix.h"
#include "evaluationrecords.h"
#include "RadarModel.hpp"
#include <utility>

/**
    * This class implements the MCDM evaluation function
    * to evaluate the utility of the frontiers.
    */
using namespace dummy;
class MCDMFunction
{

public:
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, bool use_mcdm);
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, bool use_mcdm);
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, float w_criterion_5, bool use_mcdm);
  ~MCDMFunction();
  void evaluateFrontier(Pose& p, dummy::Map* map, RFID_tools *rfid_tools, double *batteryTime);
  EvaluationRecords* evaluateFrontiers(const list< Pose >& frontiers, dummy::Map* map, double threshold, RFID_tools *rfid_tools, double *batteryTime);
  pair< Pose, double > selectNewPose(EvaluationRecords* evaluationRecords);
  string getEncodedKey(Pose& p, int value);
  EvaluationRecords* evaluateFrontiersVec(const std::vector< Pose >& frontiers,  dummy::Map& map,double threshold);

protected:

  Criterion * createCriterion(string name, double weight);
  unordered_map<string, Criterion* > criteria;
  vector<Criterion* > activeCriteria;
  WeightMatrix * matrix ;
  bool use_mcdm;
  //mutex myMutex;

};
#endif // MCDMFUNCTION_H
