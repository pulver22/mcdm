#include "criterioncomparator.h"


CriterionComparator::CriterionComparator(Pose &p) :
    p(p)
{
}
CriterionComparator::~CriterionComparator()
{

}

bool CriterionComparator::operator()(const Criterion *c1, const Criterion *c2 )
{
    
    double value1 =c1->getEvaluation(p);
   
    double value2 = c2->getEvaluation(p);
    return value1 < value2;
}


