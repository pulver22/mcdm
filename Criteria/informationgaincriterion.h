#ifndef INFORMATIONGAINCRITERION_H
#define INFORMATIONGAINCRITERION_H

#include "criterion.h"


class InformationGainCriterion : public Criterion
{
public:
    InformationGainCriterion(double weight);
    virtual ~InformationGainCriterion();
    double evaluate(Pose p, int map[][]);
private:
    void normalize(int minSensedX, int arg2);
    bool& intersect(int i, int j, int minSensedX, int minSensedY, Pose p);
};


#endif // INFORMATIONGAINCRITERION_H
