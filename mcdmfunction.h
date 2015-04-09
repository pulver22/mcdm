#ifndef MCDMFUNCTION_H
#define MCDMFUNCTION_H


#include "evaluationfunction.h"
#include "Criteria/criterion.h"
#include "Criteria/weightmatrix.h"
#include "evaluationrecords.h"

/**
    * This class implements the MCDM evaluation function
    * to evaluate the utility of the frontiers.
    */
class MCDMFunction : public EvaluationFunction
{

public:
    MCDMFunction();
    virtual ~MCDMFunction();

    virtual double evaluateFrontier(
	    const Pose* p, const Map& map);

    virtual EvaluationRecords* evaluateFrontiers(const list<Pose *> &frontiers, const Map &map);

private:

    Criterion * createCriterion(string name, double weight);        
    unordered_map<string, Criterion *> *criteria;
    WeightMatrix * matrix;
    mutex myMutex;

};
#endif // MCDMFUNCTION_H
