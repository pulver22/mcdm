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
    Ray ray = Ray();
    MCDMFunction function = MCDMFunction();
    int sensedCells = 0;
    int totalFreeCells = map.getTotalFreeCells() ;
    
    while(sensedCells < 0.9 * totalFreeCells){
	ray.findCandidatePositions();
	vector<int,int> candidatePosition = ray.getCandidatePositions();
	
	// need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
	list<Pose> frontiers;
	vector<int,int>::iterator it =candidatePosition.begin();
	for(it; it != candidatePosition.end(); it++){
	    Pose p1 = Pose(*it.first,*it.second,0,10,180);
	    Pose p2 = Pose(*it.first,*it.second,180,10,180);
	    frontiers.push_back(p1);
	    frontiers.push_back(p2);
	}
	EvaluationRecords *record = function.evaluateFrontiers(frontiers,map);
	Pose target = function.selectNewPose(record);
    }
    
}
