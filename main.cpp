#include <iostream>
#include "map.h"
#include "ray.h"
#include "mcdmfunction.h"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>



using namespace std;
using namespace dummy;

int main(int argc, char **argv) {
    
    //ifstream infile;
    //infile.open(argv[1]);
    ifstream infile("/home/pulver/Dropbox/Università/Laurea Magistrale/Thesis/testmap10.pgm");
    Map map = Map(infile,35);
    cout << map.numGridCols << " : "<<  map.numGridCols << endl;
    // Pose initialPose = map.getRobotPosition();
    
    //x,y,orientation,range,angle
    Pose initialPose = Pose(54,54,0,10,180);
    Pose target = initialPose;
    Ray ray;
    MCDMFunction function ;
    int sensedCells = 0;
    long totalFreeCells = map.getTotalFreeCells() ;
    int count = 0;
    //amount of time the robot should do nothing for scanning the environment ( final value expressed in second)
    unsigned int microseconds = 5 * 1000 * 1000 ;
    //cout << "total free cells in the main: " << totalFreeCells << endl;
    
    while(sensedCells < 0.9 * totalFreeCells){
	int x = target.getX();
	int y = target.getY();
	int orientation = target.getOrientation();
	int range = target.getRange();
	double FOV = target.getFOV();
	Map *map2 = &map;
	ray.findCandidatePositions(map2,x,y,orientation,FOV,range);
	vector<pair<int, int> > candidatePosition = ray.getCandidatePositions();
	
	//print the candidate position 
	/*
	for(vector<pair<int,int>>::iterator it=candidatePosition.begin(); it != candidatePosition.end(); it++){
	    cout << (*it).first << " : " << (*it).second << endl;
	}
	*/
	
	int newSensedCells = sensedCells + ray.getInformationGain(map2,x,y,orientation,FOV,range);
	cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;;
	if(newSensedCells == sensedCells){
	    break;
	}
	
	// need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
	list<Pose> frontiers;
	vector<pair<int, int> >::iterator it =candidatePosition.begin();
	for(it; it != candidatePosition.end(); it++){
	    Pose p1 = Pose((*it).first,(*it).second,0,10,180);
	    Pose p2 = Pose((*it).first,(*it).second,180,10,180);
	    Pose p3 = Pose((*it).first,(*it).second,90,10,180);
	    Pose p4 = Pose((*it).first,(*it).second,270,10,180);
	    frontiers.push_back(p1);
	    frontiers.push_back(p2);
	    //frontiers.push_back(p3);
	    //frontiers.push_back(p4);
	}
	
	
	EvaluationRecords *record = function.evaluateFrontiers(frontiers,map);
	cout << "Evaluation Record obtained" << endl;
	target = function.selectNewPose(record);
	ray.performSensingOperation(map2,x,y,orientation,FOV,range);
	cout << "Round : " << count<< endl;
	count = count + 1;
	sensedCells = newSensedCells;
	
	//ATTENTION:NEED TO BE IMPLEMENTED A THREAD TO STOP EXECUTION TO ALLOW ROBOT TO SCAN FOR GAS
	//NOTE: not requested for testing purpose
	usleep(microseconds);
    }
    
    cout << "Map explored" << endl;
}
