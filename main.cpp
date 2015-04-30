#include <iostream>
#include "map.h"
#include "ray.h"
#include "newray.h"
#include "mcdmfunction.h"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>



using namespace std;
using namespace dummy;

int main(int argc, char **argv) {
    
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
    
    Ray ray;
    MCDMFunction function ;
    int sensedCells = 0;
    long totalFreeCells = map.getTotalFreeCells() ;
    int count = 0;
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
	    target = function.selectNewPose(record);
	    
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
	//Map *map2 = &map;
	
	cout << "-------------------------------------------------" << endl;
	cout << "Round : " << count<< endl;
	int newSensedCells = sensedCells + ray.getInformationGain(map,x,y,orientation,FOV,range);
	cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;

	ray.performSensingOperation(map,x,y,orientation,FOV,range);
	ray.findCandidatePositions(map,x,y,orientation,FOV,range);
	vector<pair<int, int> >candidatePosition = ray.getCandidatePositions();
	ray.emptyCandidatePositions();
	
	if(candidatePosition.size() == 0) {
	    cout << "No other candidate position" << endl;
	    exit(0);
	}
	
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
	cout << "Candidate position: " << candidatePosition.size() << endl;
	cout <<"Frontiers: "<<  frontiers.size() << endl;
	EvaluationRecords *record = function.evaluateFrontiers(frontiers,map);
	//cout << "Record: " << record->size() << endl;
	cout << "Evaluation Record obtained" << endl;
	target = function.selectNewPose(record);
	
	
	count = count + 1;
	sensedCells = newSensedCells;
	
	//NOTE: not requested for testing purpose
	//usleep(microseconds);
	
	frontiers.clear();
	candidatePosition.clear();
	delete record;
	}
    }
    
    cout << "Map explored" << endl;
}
