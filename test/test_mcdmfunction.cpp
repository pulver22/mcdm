#define BOOST_TEST_MODULE MyTest
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "pose.h"
#include <cmath>
#include <iostream>
#include "mcdmfunction.h"
#include <fstream>

using namespace std;
using namespace import_map;

BOOST_AUTO_TEST_CASE( test_mcdmfunction )
{
    MCDMFunction function;
    ifstream mapURI("willow-full-bw.pgm");
    Map map = Map(mapURI,100);
    Pose p = Pose(2,3,90,5,180);
   
}