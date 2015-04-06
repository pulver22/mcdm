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
   int* intersect(int p1x, int p1y, int p2x, int p2y, Pose p);
};


#endif // INFORMATIONGAINCRITERION_H
