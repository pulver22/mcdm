#include "mcdmfunction.h"
#include "explorationconstants.h"
#include "Criteria/criterion.h"
#include "Criteria/criteriaName.h"
#include "Criteria/traveldistancecriterion.h"
#include "Criteria/informationgaincriterion.h"
#include "Criteria/RFIDCriterion.h"
#include "Criteria/sensingtimecriterion.h"
#include "Criteria/batterystatuscriterion.h"
#include "Criteria/mcdmweightreader.h"
#include "Criteria/criterioncomparator.h"
#include <string>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include "newray.h"


using namespace std;
using namespace dummy;


MCDMFunction::MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, bool use_mcdm)
{
  this->use_mcdm = use_mcdm;
  // Initialization ad-hoc: create a weightmatrix for 3 criteria with predefined weight
  MCDMWeightReader reader;
  //cout << "test" << endl;
  matrix = reader.getMatrix(w_criterion_1, w_criterion_2, w_criterion_3);
  //cout << "test2" << endl;

  // get the list of all criteria to be considered
  list<string> listCriteria = matrix->getKnownCriteria();
  for (list<string>::iterator it = listCriteria.begin(); it != listCriteria.end(); ++it) {
    string name = *it;
    // retrieve the weight of the criterion using the encoded version of the name
    double weight = matrix->getWeight(matrix->getNameEncoding(name));
    Criterion *c = createCriterion(name, weight);
    if (c != NULL) {
      criteria.emplace(name, c);
    }
  }

}


MCDMFunction::MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, bool use_mcdm)
{
  this->use_mcdm = use_mcdm;

  // Initialization ad-hoc: create a weightmatrix for 3 criteria with predefined weight
  MCDMWeightReader reader;
  //cout << "test" << endl;
  matrix = reader.getMatrix(w_criterion_1, w_criterion_2, w_criterion_3, w_criterion_4);

  // get the list of all criteria to be considered
  list<string> listCriteria = matrix->getKnownCriteria();
  for (list<string>::iterator it = listCriteria.begin(); it != listCriteria.end(); ++it) {
    string name = *it;
    // retrieve the weight of the criterion using the encoded version of the name
    double weight = matrix->getWeight(matrix->getNameEncoding(name));
    Criterion *c = createCriterion(name, weight);
    if (c != NULL) {
      criteria.emplace(name, c);
    }
  }

}

MCDMFunction::MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, float w_criterion_5, bool use_mcdm)
{
  this->use_mcdm = use_mcdm;

  // Initialization ad-hoc: create a weightmatrix for 3 criteria with predefined weight
  MCDMWeightReader reader;
  //cout << "test" << endl;
  matrix = reader.getMatrix(w_criterion_1, w_criterion_2, w_criterion_3, w_criterion_4, w_criterion_5);

  // get the list of all criteria to be considered
  list<string> listCriteria = matrix->getKnownCriteria();
  for (list<string>::iterator it = listCriteria.begin(); it != listCriteria.end(); ++it) {
    string name = *it;
    // retrieve the weight of the criterion using the encoded version of the name
    double weight = matrix->getWeight(matrix->getNameEncoding(name));
    Criterion *c = createCriterion(name, weight);
    if (c != NULL) {
      criteria.emplace(name, c);
    }
  }

}

MCDMFunction::~MCDMFunction() {
  //delete matrix;

}

//Create a criterion starting from its name and weight
Criterion *MCDMFunction::createCriterion(string name, double weight) {
  Criterion *toRet = NULL;
  if (name == (SENSING_TIME)) {
    toRet = new SensingTimeCriterion(weight);
  } else if (name == (INFORMATION_GAIN)) {
    toRet = new InformationGainCriterion(weight);
  } else if (name == (TRAVEL_DISTANCE)) {
    toRet = new TravelDistanceCriterion(weight);
  } else if (name == (RFID_READING)) {
    toRet = new RFIDCriterion(weight);
  }else if (name == (BATTERY_STATUS)) {
    toRet = new BatteryStatusCriterion(weight);
  }
  return toRet;
}

// For a candidate frontier, calculate its evaluation regarding to considered criteria and put it in the evaluation record (through
//the evaluate method provided by Criterion class)rfid
void MCDMFunction::evaluateFrontier(Pose &p, dummy::Map *map, RFID_tools *rfid_tools, double *batteryTime) {

  for (int i = 0; i < activeCriteria.size(); i++) {
    Criterion *c = activeCriteria.at(i);
    // cout << "Name: " << c->getName() << endl;
    c->evaluate(p, map, rfid_tools, batteryTime);
  }
}


// Scan a list of candidate positions,then apply the choquet fuzzy algorithm
EvaluationRecords *
MCDMFunction::evaluateFrontiers(const std::list<Pose> *frontiers, dummy::Map *map,
   double threshold, RFID_tools *rfid_tools, double *batteryTime, bool *explorationCompleted) {

  Pose f;
  //Clean the last evaluation
  //NOTE: probably working
  unordered_map<string, Criterion *>::iterator it;
  for (it = criteria.begin(); it != criteria.end(); it++) {
    std::pair<string, Criterion *> pair = *it;
    (criteria.at(pair.first))->clean();
  }


  // listActiveCriteria contains the name of the criteria while "criteria struct" contain the pairs <name, criterion>
  vector<string> listActiveCriteria = matrix->getActiveCriteria();
  // cout << "List activeCriteria: " << endl;
  for (vector<string>::iterator it = listActiveCriteria.begin(); it != listActiveCriteria.end(); it++) {
    activeCriteria.push_back(criteria[*it]);
    // cout << "   " << criteria[*it] << endl;
  }

  //Evaluate the frontiers
  list<Pose>::const_iterator it2;
  for (it2 = frontiers->begin(); it2 != frontiers->end(); it2++) {
    f = *it2;
    evaluateFrontier(f, map, rfid_tools, batteryTime);
  }

  //Normalize the values
  for (vector<Criterion *>::iterator it = activeCriteria.begin(); it != activeCriteria.end(); ++it) {
    (*it)->normalize();
  }

  //Create the EvaluationRecords
  EvaluationRecords *toRet = new EvaluationRecords();
  


  // analyze every single frontier f, and add in the evaluationRecords <frontier, evaluation>
  for (list<Pose>::const_iterator i = frontiers->begin(); i != frontiers->end(); i++) {

    // cout <<"---------------------NEW FRONTIER -------------------"<<endl;
    f = *i;

    // order criteria depending on the considered frontier
    sort(activeCriteria.begin(), activeCriteria.end(), CriterionComparator(f));

    //apply the choquet integral
    Criterion *lastCrit = NULL;
    double finalValue = 0.0;
    // cout << "\n==== New frontier ====" << endl;
    bool no_info_gain = false;
    
    // WEIGHTED AVG
    if (this->use_mcdm == false){
      for (vector<Criterion *>::iterator k = activeCriteria.begin(); k != activeCriteria.end(); k++) {
        Criterion *c = NULL;
        double weight = 0.0;
        list<string> names;
        names.push_back((*k)->getName());
        // Get the weight of the single criterion
        weight = matrix->getWeight(names);
        finalValue += (*k)->getWeight() * (*k)->getEvaluation(f);
        if ((*k)->getName().compare("informationGain") == 0){
          if ((*k)->getEvaluation(f) == 0){
            no_info_gain = true;
          }
        }
      }
    }else{
      //MCDM
      for (vector<Criterion *>::iterator k = activeCriteria.begin(); k != activeCriteria.end(); k++) {
        Criterion *c = NULL;
        double weight = 0.0;
        list<string> names;
        for (vector<Criterion *>::iterator j = k; j != activeCriteria.end(); j++) {
          Criterion *next = (*j);
          names.push_back(next->getName()); // The list of criteria whose evaluation is >= than the one's considered
        }
        weight = matrix->getWeight(names);
        if (k == activeCriteria.begin()) {
          c = (*k);
          finalValue += c->getEvaluation(f) * weight;
        } else {
          c = (*k);
          double tmpValue = c->getEvaluation(f) - lastCrit->getEvaluation(f);
          finalValue += tmpValue * weight;
        }
        lastCrit = c;

        if (c->getName().compare("informationGain") == 0){
          if (c->getEvaluation(f) == 0){
            no_info_gain = true;
          }
        }
      }


    }

    // If we are still exploring the environment, add the frontier only
    // if it leads to some map coverage
    if (finalValue > threshold and no_info_gain == false and *explorationCompleted==false){
      toRet->putEvaluation(f, finalValue);
    }
      
    // If the map is already explored and we are focusing on entropy, add the frontier
    // as long as it has a good utility
    if (finalValue > threshold and *explorationCompleted==true){
      toRet->putEvaluation(f, finalValue);
    }
  
  }
    

    

  activeCriteria.clear();
  return toRet;
}

pair<Pose, double> MCDMFunction::selectNewPose(EvaluationRecords *evaluationRecords) {
  Pose newTarget;
  double value = 0;
  unordered_map<string, double> evaluation = evaluationRecords->getEvaluations();
  for (unordered_map<string, double>::iterator it = evaluation.begin(); it != evaluation.end(); it++) {
    string tmp = (*it).first;
    Pose p = evaluationRecords->getPoseFromEncoding(tmp);
    if (value <= (*it).second) {
      newTarget = p;
      value = (*it).second;
    }//else continue;
  }
  pair<Pose, double> result = make_pair(newTarget, value);

  // i switch x and y to allow debugging graphically looking the image
//     cout << "New target : " << "x = "<<newTarget.getY() <<", y = "<< newTarget.getX() << ", orientation = "
// 	   <<newTarget.getOrientation() << ", Evaluation: "<< value << endl;
  return result;
}

string MCDMFunction::getEncodedKey(Pose &p, int value) {
  string key;
  //value = 0 : encode everything
  //value = 1 : encode x,y,orientation, take first
  //value = 2 : encode x,y,orientation, take multiple time
  if (value == 0 or value == 1) {
    key = to_string(p.getX()) + "/" + to_string(p.getY()) + "/" + to_string(p.getOrientation()) + "/" +
          to_string(p.getRange()) + "/" + to_string(p.getFOV()) + "/" + to_string(p.getScanAngles().first) + "/" +
          to_string(p.getScanAngles().second);
  // } else if (value == 1) {
  //   key = to_string(p.getX()) + "/" + to_string(p.getY()) + "/" + to_string(p.getOrientation()) + "/" + "1";
  } else if (value == 2) {
    key = to_string(p.getX()) + "/" + to_string(p.getY()) + "/" + to_string(p.getOrientation()) + "/" + "2";
  }
  return key;
}

void MCDMFunction::updateCriteria(double w_criterion_1, double w_criterion_2,
                                  double w_criterion_3, double w_criterion_4,
                                  double w_criterion_5) {
  // Initialization ad-hoc: create a weightmatrix for 3 criteria with predefined
  // weight
  MCDMWeightReader reader;
  // cout << "test" << endl;
  matrix = reader.getMatrix(w_criterion_1, w_criterion_2, w_criterion_3,
                            w_criterion_4, w_criterion_5);

  //Remove all existing criteria
  this->criteria.clear();
  
  // get the list of all criteria to be considered
  list<string> listCriteria = matrix->getKnownCriteria();
  for (list<string>::iterator it = listCriteria.begin();
       it != listCriteria.end(); ++it) {
    string name = *it;
    // retrieve the weight of the criterion using the encoded version of the
    // name
    double weight = matrix->getWeight(matrix->getNameEncoding(name));
    Criterion *c = createCriterion(name, weight);
    if (c != NULL) {
      this->criteria.emplace(name, c);
    }
  }
}
