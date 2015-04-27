#define BOOST_TEST_MODULE MyTest
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "pose.h"
#include "Criteria/criteriaName.h"
#include "Criteria/weightmatrix.h"
#include "Criteria/sensingtimecriterion.h"
#include "Criteria/traveldistancecriterion.h"
#include "Criteria/informationgaincriterion.h"
#include <cmath>
#include <iostream>

using namespace std;

BOOST_AUTO_TEST_CASE( test_weightmatrix )
{
    //create a weightmatrix for 3 criteria
    WeightMatrix matrix(3);
    matrix.insertSingleCriterion(INFORMATION_GAIN,0.5,true);
    matrix.insertSingleCriterion(TRAVEL_DISTANCE,0.2, true);
    matrix.insertSingleCriterion(SENSING_TIME,0.3, true);
    string str1(INFORMATION_GAIN);
    string str2(TRAVEL_DISTANCE);
    string str3(SENSING_TIME);
    list<string> list1 ;
    list<string> list2 ;
    list<string> list3 ;
    list<string> list4;
    list<string> listCasual;
    list1.push_back(str1);
    list1.push_back(str2);
    list2.push_back(str1);
    list2.push_back(str3);
    list3.push_back(str2);
    list3.push_back(str3);
    list4.push_back(str1);
    list4.push_back(str2);
    list4.push_back(str3);
    listCasual.push_back(str2);
    listCasual.push_back(str1);
    matrix.insertCombinationWeight(list1,0.8);
    matrix.insertCombinationWeight(list2,0.6);
    matrix.insertCombinationWeight(list3,0.6);
    matrix.insertCombinationWeight(list4,1);
    
    cout << "1) check the encoding of single criteria" << endl;
    string encInforGain = matrix.getNameEncoding(INFORMATION_GAIN);
    string encTravDist = matrix.getNameEncoding(TRAVEL_DISTANCE);
    string encSensTime = matrix.getNameEncoding(SENSING_TIME);
    BOOST_CHECK_EQUAL(encInforGain,"A");
    BOOST_CHECK_EQUAL(encTravDist,"B");
    BOOST_CHECK_EQUAL(encSensTime,"C");
    
    cout << "2) check the encoding of combined criteria " << endl;
    string encList1 = matrix.computeNamesEncoding(list1);
    string encList2 = matrix.computeNamesEncoding(list2);
    string encList3 = matrix.computeNamesEncoding(list3);
    string encList4 = matrix.computeNamesEncoding(list4);
    BOOST_CHECK_EQUAL(encList1,"AB");
    BOOST_CHECK_EQUAL(encList2,"AC");
    BOOST_CHECK_EQUAL(encList3, "BC");
    BOOST_CHECK_EQUAL(encList4, "ABC");
    
    cout << "3) check the encoding of combined criteria in casual order" << endl;
    string encListCasual = matrix.computeNamesEncoding(listCasual);
    BOOST_CHECK_EQUAL(encListCasual,"AB");
    
    cout << "4) chech the activation of criteria before and after a changing state" << endl;
    vector<string> activeCriteria = matrix.getActiveCriteria();
    
    for(int it = 0; it < activeCriteria.size(); it++){
	if(it ==0){
	    string tmp = activeCriteria.at(it);
	    //cout << tmp << endl;
	    BOOST_CHECK_EQUAL(tmp, "informationGain");
	}else if (it == 1){
	    string tmp = activeCriteria.at(it);
	    //cout << tmp << endl;
	    BOOST_CHECK_EQUAL(tmp, "travelDistance");  
	}else {
	    string tmp = activeCriteria.at(it);
	    //cout << tmp << endl;
	    BOOST_CHECK_EQUAL(tmp, "sensingTime");
	}
    }
    
    int sizeActiveCriteria = matrix.getNumOfActiveCriteria();
    BOOST_CHECK_EQUAL(sizeActiveCriteria,3);
    matrix.changeCriteriaActivation(str1,false);
    sizeActiveCriteria = matrix.getNumOfActiveCriteria();
    BOOST_CHECK_EQUAL(sizeActiveCriteria,2);
    
    cout << "5) check the insertion of criteria" << endl;
    list<string> knownCriteria = matrix.getKnownCriteria();
    BOOST_CHECK_EQUAL(knownCriteria.size(),3);
    list<string>::iterator it = knownCriteria.begin();
   
    cout << "6) check the correct insertion of weights" << endl;
    double weightTravel = matrix.getWeight(encTravDist);
    double weightInfo = matrix.getWeight(encInforGain);
    double weightSensing = matrix.getWeight(encSensTime);
    double weightInfoTravel = matrix.getWeight(list1);
    cout << "weightInfoTravel = " << weightInfoTravel << endl;
    double weightInfoSensing = matrix.getWeight(list2);
    cout << "weightInfoSensing = " << weightInfoSensing << endl;
    double weightTravelSensing = matrix.getWeight(list3);
    cout << "weightTravelSensing = " << weightTravelSensing << endl;
    double weightAll = matrix.getWeight(list4);
    cout << "weightAll = " << weightAll << endl;
 
    BOOST_CHECK_EQUAL(weightInfo,0.5);
    BOOST_CHECK_EQUAL(weightTravel,0.2);
    BOOST_CHECK_EQUAL(weightSensing, 0.3);
    BOOST_CHECK_EQUAL(weightInfoTravel,0.8);
    BOOST_CHECK_EQUAL(weightInfoSensing,0.6);
    BOOST_CHECK_EQUAL(weightTravelSensing, 0.6);
    BOOST_CHECK_EQUAL(weightAll, 1);
    
}

//EVERYTHING WORK LIKE A CHARM