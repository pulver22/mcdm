#define BOOST_TEST_MODULE MyTest
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "pose.h"
#include <cmath>
#include <iostream>

using namespace std;

BOOST_AUTO_TEST_CASE( test_pose )
{
    Pose p = Pose (2,4,40,5,30) ;
    Pose p2 = Pose (1,3,50,6,20);

    cout << "check creation of a pose" << endl;
    BOOST_CHECK_EQUAL( p.getX(), 2 );
    BOOST_CHECK_EQUAL(p.getY(),4);
    BOOST_CHECK_EQUAL(p.getOrientation(),40);
    BOOST_CHECK_EQUAL(p.getR(), 5);
    BOOST_CHECK_EQUAL(p.getPhi(), 30);
 
    cout << "check distance" << endl ;
    BOOST_CHECK_EQUAL(p.getDistance(p2),sqrt(2));
}

// EVERYTHING WORK LIKE A CHARM