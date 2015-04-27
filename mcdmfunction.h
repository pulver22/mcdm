#ifndef MCDMFUNCTION_H
#define MCDMFUNCTION_H
#include "Criteria/criterion.h"
#include "Criteria/weightmatrix.h"
#include "evaluationrecords.h"
#include <utility>

/**
    * This class implements the MCDM evaluation function
    * to evaluate the utility of the frontiers.
    */
class MCDMFunction 
{

public:
    MCDMFunction();
    virtual ~MCDMFunction();
    virtual double evaluateFrontier(Pose p, import_map::Map &map);
    virtual EvaluationRecords* evaluateFrontiers(list< Pose >& frontiers, import_map::Map &map);
    virtual Pose selectNewPose(EvaluationRecords* evaluationRecords);
    
protected:

    Criterion * createCriterion(string name, double weight);        
    unordered_map<string, Criterion* > criteria;
    vector<Criterion* > activeCriteria;
    WeightMatrix * matrix ;
    mutex myMutex;

};
#endif // MCDMFUNCTION_H
