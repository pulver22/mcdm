#define BOOST_TEST_MODULE MyTest
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "graphpose.h"
#include <cmath>
#include <iostream>


using namespace std;

BOOST_AUTO_TEST_CASE( test_graphpose )
{
    cout << "1) check the creation of a empty graph" << endl;
    GraphPose graph = GraphPose();
    int size = (graph.getGraph()).size();
    BOOST_CHECK_EQUAL( size ,0);
    
    
    Pose p1 = Pose(1,2,90,4,30);
    Pose p2 = Pose(5,1,90,5,30);
    Pose p3 = Pose(6,8,180,4,30);
    Pose actualPose = Pose(3,3,270,6,20);
    
    
    cout << "2) check the insertion of a new pose in the graph" << endl;
    graph.addPose(p1,actualPose);
    graph.addPose(p2,p1);
    graph.addPose(p3,p2);
    //cout << "test string"<< endl;
    int  size2 = (graph.getGraph()).size();
    BOOST_CHECK_EQUAL(size2,4);
    
    
    cout << "3) check the correct destionation" << endl;
    list<Edge> tmpList = graph.getKnownDestination(p2);
    auto it =tmpList.begin();
    advance(it, 1);
    Edge tmpEdge = *it;
    Pose tmpPose = tmpEdge.destination;
    BOOST_CHECK_EQUAL(tmpPose.getX(),p3.getX());
    BOOST_CHECK_EQUAL(tmpPose.getY(),p3.getY());
    
    cout << "4) check the encoding " << endl;
    string tmpStr = graph.getEncodedKey(actualPose);
    BOOST_CHECK_EQUAL(tmpStr,"3/3/270/6/20");
    
}

//EVERYTHING WORK LIKE A CHARM