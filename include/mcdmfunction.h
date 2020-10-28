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

  /**
   * Create a MCDM function using three criteria.
   * 
   * @param w_criterion_1: the weight of the first criterion
   * @param w_criterion_2: the weight of the second criterion
   * @param w_criterion_3: the weight of the third criterion
   * @param use_mcdm: is using mcdm of weighted average when selecting a frontier
   */
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, bool use_mcdm);
  
  /**
   * Create a MCDM function using four criteria.
   * 
   * @param w_criterion_1: the weight of the first criterion
   * @param w_criterion_2: the weight of the second criterion
   * @param w_criterion_3: the weight of the third criterion
   * @param w_criterion_4: the weight of the fourth criterion
   *  @param use_mcdm: is using mcdm of weighted average when selecting a frontier
   */
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, bool use_mcdm);
  
  /**
   * Create a MCDM function using five criteria.
   * 
   * @param w_criterion_1: the weight of the first criterion
   * @param w_criterion_2: the weight of the second criterion
   * @param w_criterion_3: the weight of the third criterion
   * @param w_criterion_4: the weight of the fourth criterion
   * @param w_criterion_5: the weight of the fifth criterion
   *  @param use_mcdm: is using mcdm of weighted average when selecting a frontier
   */
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, float w_criterion_5, bool use_mcdm);

  ~MCDMFunction();

  /**
   * Evaluate a single frontier based on all the criteria available.
   * 
   * @param p: the frontier to evaluate
   * @param map: the reference to the map
   * @param rfid_tools: various RFID utilities
   * @param batteryTime: the remaining battery time
   */ 
  void evaluateFrontier(Pose& p, dummy::Map* map, RFID_tools *rfid_tools, double *batteryTime);

  /**
   * Evaluate a list of frontiers based on all the criteria available.
   * 
   * @param frontiers: the list of all the frontiers available
   * @param map: the reference to the map
   * @param threshold: needed to discard not meaningful frontiers
   * @param rfid_tools: various RFID utilities
   * @param batteryTime: the remaining battery time
   * @return a record of frontiers and their associated evaluation
   */ 
  EvaluationRecords* evaluateFrontiers(const list< Pose >* frontiers, dummy::Map* map, 
    double threshold, RFID_tools *rfid_tools, double *batteryTime, bool *explorationCompleted);

  /**
   * Find the best frontier where to send the robot.
   * 
   * @param evaluationRecords: the list of all the frontiers and their evaluation
   * @return the selected new destination with its evaluation.
   */
  pair< Pose, double > selectNewPose(EvaluationRecords* evaluationRecords);

  /**
   * Convert a pose into a string
   * 
   * @param p: the pose to encode
   * @param value:  0 (encode everything), 1 (x,y,orientation - first), 2 (x,y,orientation - multiple time)
   * @return the encoding
   */
  string getEncodedKey(Pose& p, int value);

  void updateCriteria(double w_criterion_1, double w_criterion_2, double w_criterion_3, double w_criterion_4, double w_criterion_5);

protected:

  /**
   * Create a new criterion
   * 
   * @param name: the name of the criterion
   * @para weight: its importance weight
   * @return an object Criterion
   */
  Criterion * createCriterion(string name, double weight);
  unordered_map<string, Criterion* > criteria;
  vector<Criterion* > activeCriteria;
  WeightMatrix * matrix ;
  bool use_mcdm;
  //mutex myMutex;

};
#endif // MCDMFUNCTION_H
