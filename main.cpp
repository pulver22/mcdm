#include <iostream>
#include <iterator>
#include "map.h"
#include "ray.h"
#include "newray.h"
#include "mcdmfunction.h"
#include "graphpose.h"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>



using namespace std;
using namespace dummy;

int main(int argc, char **argv) {
    
    // Input : ./mcdm_online_exploration_ros ~/Desktop/map_4.pgm 80 730 880 0 20 180 1

    ifstream infile;
    infile.open(argv[1]);
    int resolution = atoi(argv[2]);
    //ifstream infile("/home/pulver/Dropbox/Università/Laurea Magistrale/Thesis/testmap10.pgm");
    Map map = Map(infile,resolution);
    cout << "Map dimension: " << map.numGridCols << " : "<<  map.numGridCols << endl;
    // Pose initialPose = map.getRobotPosition();
    
    // i switched x and y because the map's orientation inside and outside programs are different
    int initX = atoi(argv[4]);	
    int initY = atoi(argv[3]);
    int initOrientation = atoi(argv[5]);
    double initFov = atoi(argv[7]);
    int initRange = atoi(argv[6]);
    int rayType = atoi(argv[8]);
    //x,y,orientation,range,angle -
    Pose initialPose = Pose(initX,initY,initOrientation,initRange,initFov);
    Pose target = initialPose;
    Pose previous = initialPose;
    GraphPose graph;
    //testing
    vector<pair<string,list<Pose>>> graph2;
    
    Ray ray;
    MCDMFunction function ;
    int sensedCells = 0;
     int newSensedCells =0;
    long totalFreeCells = map.getTotalFreeCells() ;
    int count = 0;
    int countBT;
    //amount of time the robot should do nothing for scanning the environment ( final value expressed in second)
    unsigned int microseconds = 5 * 1000 * 1000 ;
    //cout << "total free cells in the main: " << totalFreeCells << endl;
    
    if(rayType == 0){
    cout << "OLD RAY VERSION! " << endl;
    Ray ray;
	while(sensedCells < 0.9 * totalFreeCells){
	    int x = target.getX();
	    int y = target.getY();
	    int orientation = target.getOrientation();
	    int range = target.getRange();
	    double FOV = target.getFOV();
	    
	    //Map *map2 = &map;

	    // OldRayCode
	    
	    cout << "-------------------------------------------------" << endl;
	    cout << "Round : " << count<< endl;

	    int newSensedCells = sensedCells + ray.getInformationGain(map,x,y,orientation,FOV,range);
	    cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
	    
	    ray.findCandidatePositions(map,x,y,orientation,FOV,range);
	    vector<pair<int, int> >candidatePosition = ray.getCandidatePositions();
	    ray.emptyCandidatePositions();
	   
	    if(candidatePosition.size() == 0) {
		cout << "No other candidate position" << endl;
		exit(0);
	    }
	    
	    // need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
	    list<Pose> frontiers;
	    vector<pair<int, int> >::iterator it =candidatePosition.begin();
	    for(it; it != candidatePosition.end(); it++){
		Pose p1 = Pose((*it).first,(*it).second,0,range,FOV);
		Pose p2 = Pose((*it).first,(*it).second,180,range,FOV);
		Pose p3 = Pose((*it).first,(*it).second,90,range,FOV);
		Pose p4 = Pose((*it).first,(*it).second,270,range,FOV);
		frontiers.push_back(p1);
		frontiers.push_back(p2);
		//frontiers.push_back(p3);
		//frontiers.push_back(p4);
	    }
	    cout << "Candidate position: " << candidatePosition.size() << endl;
	    cout <<"Frontiers: "<<  frontiers.size() << endl;
	    EvaluationRecords *record = function.evaluateFrontiers(frontiers,map);
	    //cout << "Record: " << record->size() << endl;
	    cout << "Evaluation Record obtained" << endl;
	    
	    target = function.selectNewPose(record).first;
	   
	    // to use with normal ray
	    ray.performSensingOperation(map,x,y,orientation,FOV,range);

	    
	    
	    count = count + 1;
	    sensedCells = newSensedCells;
	    
	    //NOTE: not requested for testing purpose
	    //usleep(microseconds);
	    
	    frontiers.clear();
	    candidatePosition.clear();
	    delete record;
	}
    }else{
	cout << "NEW RAY VERSION !"<< endl;
	NewRay ray;
	while(sensedCells < 0.9 * totalFreeCells){
	    int x = target.getX();
	    int y = target.getY();
	    int orientation = target.getOrientation();
	    int range = target.getRange();
	    double FOV = target.getFOV();
	    string actualPose = graph.getEncodedKey(target);
	    
	    
	    //Map *map2 = &map;
	    
	    cout << "-------------------------------------------------" << endl;
	    cout << "Round : " << count<< endl;
	    newSensedCells = sensedCells + ray.getInformationGain(map,x,y,orientation,FOV,range);
	    cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;

	    ray.performSensingOperation(map,x,y,orientation,FOV,range);
	    ray.findCandidatePositions(map,x,y,orientation,FOV,range);
	    vector<pair<int, int> >candidatePosition = ray.getCandidatePositions();
	    ray.emptyCandidatePositions();
	    
	    
	    if(candidatePosition.size() == 0) {
		
		cout << "No other candidate position" << endl;
		cout << "----- BACKTRACKING -----" << endl;
		/*
		cout << "----- BACKTRACKING -----" << endl;
	    
		// get the position reachable from the previous one
		countBT = countBT -2;
		pair<string, list<Pose>> consideredCell = graph2.at(countBT);
		string tmpPose = consideredCell.first;
		cout <<"new pose : "<< tmpPose <<endl;
		list<Pose> exploredFrontiers = consideredCell.second;
		//cout << "Lenght of frontiers : " << exploredFrontiers.size() << endl;
		EvaluationRecords *record = function.evaluateFrontiers(exploredFrontiers,map);
		//cout << "Record dimension: " << record->size() << endl;
		vector<Pose> vectTemp = record->getFrontiers();
		// print the candidate position
		if(record->size() != 0){
		    for(int i =0; i < vectTemp.size();i++){
			Pose tmp = vectTemp.at(i);
			string str = record->getEncodedKey(tmp);
			//cout << "ciao" << endl;
			//cout << str <<endl;
		    }
		    pair<Pose,double> tmp = function.selectNewPose(record);
		    target = tmp.first;
		     //cout << "New target : " << "x = "<<target.getY() <<", y = "<< target.getX() << ", orientation = " 
			//    <<target.getOrientation() << ", Evaluation: "<< tmp.second << endl;
		    //std::pair<string,list<Pose>> pair = make_pair(tmpPose,exploredFrontiers);
		}else {
		    cout << "prova" <<endl;
		    countBT = countBT -1;
		    consideredCell = graph2.at(countBT);
		    string targetString = consideredCell.first;
		    target = record->getPoseFromEncoding(targetString);  
		}
		
		count = count + 1;
		//countBT = count;
		exploredFrontiers.clear();
		delete record;
		*/
		
		countBT = countBT -1;
		string targetString = graph2.at(countBT).first;
		graph2.pop_back();
		EvaluationRecords record;
		target = record.getPoseFromEncoding(targetString);
		cout << "[BT]No significative position reachable. Come back to previous position" << endl;
		cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
		count = count + 1;
		//exit(0);
		
		    
	    }else{
		//Can cause the navigation to stop: to fix this problem we should avoid candidate position with infoGain = 0;
		
		//   if(newSensedCells == sensedCells){
		//	break;
		//    }
		
		    
		// need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
		list<Pose> frontiers;
		vector<pair<int, int> >::iterator it =candidatePosition.begin();
		for(it; it != candidatePosition.end(); it++){
		    Pose p1 = Pose((*it).first,(*it).second,0,range,FOV);
		    Pose p2 = Pose((*it).first,(*it).second,180,range,FOV);
		    Pose p3 = Pose((*it).first,(*it).second,90,range,FOV);
		    Pose p4 = Pose((*it).first,(*it).second,270,range,FOV);
		    frontiers.push_back(p1);
		    frontiers.push_back(p2);

		    //frontiers.push_back(p3);
		    //frontiers.push_back(p4);
		}
		
		cout << "Graph dimension : " << graph2.size() << endl;
		cout << "Candidate position: " << candidatePosition.size() << endl;
		cout <<"Frontiers: "<<  frontiers.size() << endl;
		EvaluationRecords *record = function.evaluateFrontiers(frontiers,map);
		//cout << "Record: " << record->size() << endl;
		cout << "Evaluation Record obtained" << endl;
		previous = target;
		
		if(record->size() != 0){
		    std::pair<string,list<Pose>> pair = make_pair(actualPose,frontiers);
		    graph2.push_back(pair);
		    std::pair<Pose,double> result = function.selectNewPose(record);
		    target = result.first;
		    count = count + 1;
		    countBT = graph2.size();
		}else {
		    countBT = countBT -1;
		    string targetString = graph2.at(countBT).first;
		    target = record->getPoseFromEncoding(targetString);
		    graph2.pop_back();
		    cout << "No significative position reachable. Come back to previous position" << endl;
		    cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
		    count = count + 1;
		}
	
		sensedCells = newSensedCells;
		
		//NOTE: not requested for testing purpose
		//usleep(microseconds);
		
		frontiers.clear();
		candidatePosition.clear();
		delete record;
	    }
	}
    }
    
    cout << "Map explored" << endl;
}
