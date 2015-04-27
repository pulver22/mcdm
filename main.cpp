#include <iostream>
#include "map.h"
#include "ray.h"
#include "mcdmfunction.h"


using namespace std;
using namespace import_map;

int main(int argc, char **argv) {
    
    ifstream infile;
    infile.open(argv[1]);
    Map map = Map(infile,100);
    Pose initialPose = map.getRobotPosition();
    Pose target = initialPose;
    Ray ray;
    MCDMFunction function ;
    int sensedCells = 0;
    int totalFreeCells = map.getTotalFreeCells() ;
    
    while(sensedCells < 0.9 * totalFreeCells){
	int count = 0;
	int x = target.getX();
	int y = target.getY();
	int orientation = target.getOrientation();
	int range = target.getR();
	double FOV = target.getPhi();
	ray.findCandidatePositions(map,x,y,orientation,FOV,range);
	vector<pair<int, int> > candidatePosition = ray.getCandidatePositions();
	sensedCells = sensedCells + ray.getInformationGain(map,x,y,orientation,FOV,range);
	
	// need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
	list<Pose> frontiers;
	vector<pair<int, int> >::iterator it =candidatePosition.begin();
	for(it; it != candidatePosition.end(); it++){
	    Pose p1 = Pose((*it).first,(*it).second,0,10,180);
	    Pose p2 = Pose((*it).first,(*it).second,180,10,180);
	    frontiers.push_back(p1);
	    frontiers.push_back(p2);
	}
	EvaluationRecords *record = function.evaluateFrontiers(frontiers,map);
	target = function.selectNewPose(record);
	cout << "Round : " << count<< endl;
	count++;
    }
    
    cout << "Map explored" << endl;
}
