#define BOOST_TEST_MODULE MyTest
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "pose.h"
#include <cmath>
#include <iostream>
#include "evaluationrecords.h"

using namespace std;

BOOST_AUTO_TEST_CASE( test_pose )
{
    cout << "1) check creation of the record for the evaluations" << endl;
    EvaluationRecords record;
    BOOST_CHECK_EQUAL(record.size(),0);
    Pose p1 = Pose(2,4,270,5,45);
    Pose p2 = Pose(3,5,90,7,30);
    record.putEvaluation(p1,43);
    BOOST_CHECK_EQUAL(record.size(), 1);
    
    cout << "2) check the correct insertion of the pose " << endl;
    BOOST_CHECK_EQUAL(record.contains(p1),true);
    BOOST_CHECK_EQUAL(record.contains(p2),false);
    
    cout << "3) check the retrieving of a proper frontier " << endl;
    double value = record.getEvaluation(p1);
    BOOST_CHECK_EQUAL(value,43);
    
    cout << "4) check the correct deletion of a frontier " << endl;
    record.putEvaluation(p2,27);
    BOOST_CHECK_EQUAL(record.size(),2);
    record.removeFrontier(p1);
    BOOST_CHECK_EQUAL(record.size(),1);
    
    
    cout << "5) check the normalization " << endl;
    record.putEvaluation(p1,43);
    record.normalize();
    double value1 = record.getEvaluation(p1);
    double value2 = record.getEvaluation(p2);
    BOOST_CHECK_EQUAL(value1,1);
    BOOST_CHECK_EQUAL(value2,0);
    
    cout << "6) check the retrieving of all the evaluations (pair <pose,evaluation>)" << endl;
    unordered_map<string,double> evaluations = record.getEvaluations();
    //cout << evaluations.size() << endl;
    unordered_map<string,double>::iterator it = evaluations.begin();
    std::pair<string,double> pair = *it;
    //cout << pair.first << endl;
    Pose ptest =record.getPoseFromEncoding(pair.first);
    BOOST_CHECK_EQUAL(p2.isEqual(ptest),true);
    
    cout << "7) check the retriving of the Pose's list" << endl;
    vector<Pose> pose = record.getFrontiers();
    BOOST_CHECK_EQUAL(pose.size(),2);
    BOOST_CHECK_EQUAL(p2.isEqual(pose.at(0)),true);
    BOOST_CHECK_EQUAL(p1.isEqual(pose.at(1)),true);
    
}

//EVERYTHING WORK LIKE  A CHARM