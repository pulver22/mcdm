#define BOOST_TEST_MODULE MyTest
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "pose.h"
#include <cmath>
#include <iostream>
#include "Criteria/criterion.h"
#include "mcdmfunction.h"
#include "Criteria/criterioncomparator.h"
#include <Criteria/sensingtimecriterion.h>
#include <Criteria/traveldistancecriterion.h>

using namespace std;

BOOST_AUTO_TEST_CASE( test_criterioncomparator )
{
    // creation of a test position
    Pose p = Pose(2,4,40,5,30) ;
    // creation of two candidate criteria
    SensingTimeCriterion sensingTime(0.5);
    TravelDistanceCriterion travelDistance(0.8);
    // insertion of two criteria's evaluation
    sensingTime.insertEvaluation(p,0.4);
    travelDistance.insertEvaluation(p,0.7);
    
    /*
    //to be finished...
    CriterionComparator cp(p);
    BOOST_CHECK_EQUAL( CriterionComparator(p), true) ;
    */
    
}