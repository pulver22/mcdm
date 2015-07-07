#include <algorithm>
#include <iostream>
#include <iterator>
#include "map.h"
#include "newray.h"
#include "mcdmfunction.h"
#include "Criteria/traveldistancecriterion.h"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>



using namespace std;
using namespace dummy;
bool contains(std::list< Pose >& list, Pose& p);
void cleanPossibleDestination2(std::list< Pose > &possibleDestinations, Pose& p);


int main(int argc, char **argv) {
    
    // Input : ./mcdm_online_exploration_ros ./../Maps/map_RiccardoFreiburg_1m2.pgm 100 75 5 0 15 180 0.95 0.12
    // resolution x y orientation range centralAngle precision threshold

    ifstream infile;
    infile.open(argv[1]);
    int resolution = atoi(argv[2]);
    Map map = Map(infile,resolution);
    cout << "Map dimension: " << map.getNumGridCols() << " : "<<  map.getNumGridRows() << endl;
    
    // i switched x and y because the map's orientation inside and outside programs are different
    long  initX = atoi(argv[4]);	
    long initY = atoi(argv[3]);
    int initOrientation = atoi(argv[5]);
    double initFov = atoi(argv[7] );
    initFov = initFov * PI /180;
    int initRange = atoi(argv[6]);
    double precision = atof(argv[8]);
    double threshold = atof(argv[9]);
    //x,y,orientation,range,FOV
    Pose initialPose = Pose(initX,initY,initOrientation,initRange,initFov);
    Pose p1 = Pose(initX,initY,0,initRange,initFov);
    Pose p2 = Pose(initX,initY,90,initRange,initFov);
    Pose p3 = Pose(initX,initY,180,initRange,initFov);
    Pose p4 = Pose(initX,initY,270,initRange,initFov);
    Pose target = initialPose;
    Pose previous = initialPose;
    long numConfiguration =0;
    vector<pair<string,list<Pose>>> graph2;
    NewRay ray;
    MCDMFunction function;
    long sensedCells = 0;
    long newSensedCells =0;
    long totalFreeCells = map.getTotalFreeCells() ;
    int count = 0;
    double travelledDistance = 0;
    int numOfTurning = 0;
    unordered_map<string,int> visitedCell;
    vector<string>history;
    history.push_back(function.getEncodedKey(target,1));
    //amount of time the robot should do nothing for scanning the environment ( final value expressed in second)
    unsigned int microseconds = 5 * 1000 * 1000 ;
    list<Pose> unexploredFrontiers;
    list<Pose> tabuList;
    list<Pose> nearCandidates;
    bool btMode = false;
    Astar astar;

    
    while(sensedCells < precision * totalFreeCells ){
	if(btMode == false){
	    long x = target.getX();
	    long y = target.getY();
	    int orientation = target.getOrientation();
	    int range = target.getRange();
	    double FOV = target.getFOV();
	    string actualPose = function.getEncodedKey(target,0);
	    map.setCurrentPose(target);
	    string path = astar.pathFind(target.getX(),target.getY(),previous.getX(),previous.getY(),map);
	    travelledDistance = travelledDistance + astar.lenghtPath(path);
	    numOfTurning = numOfTurning + astar.getNumberOfTurning(path);
	    string encoding = to_string(target.getX()) + to_string(target.getY());
	    visitedCell.emplace(encoding,0);
	    
	    
	    
	    cout << "-----------------------------------------------------------------"<<endl;
	    cout << "Round : " << count<< endl;
	    newSensedCells = sensedCells + ray.getInformationGain(map,x,y,orientation,FOV,range);
	    cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
	    target.setScanAngles(ray.getSensingTime(map,x,y,orientation,FOV,range));
	    cout << "MinPhi: " << target.getScanAngles().first << " MaxPhi: " << target.getScanAngles().second << endl;
	    ray.performSensingOperation(map,x,y,orientation,FOV,range, target.getScanAngles().first, target.getScanAngles().second);

	    ray.findCandidatePositions(map,x,y,orientation,FOV,range);
	    vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
	    ray.emptyCandidatePositions();
	    
	    
	    if(candidatePosition.size() == 0) {
		
		cout << "No other candidate position" << endl;
		cout << "----- BACKTRACKING -----" << endl;
		
		if (graph2.size() > 1){
		    
		    //get the last position in the graph and then remove it
		    
		    string targetString = graph2.at(graph2.size()-1).first;
		    graph2.pop_back();
		    
		  
		    
		    EvaluationRecords record;
		    target = record.getPoseFromEncoding(targetString);
		    history.push_back(function.getEncodedKey(target,2));
		    cout << "[BT]No significative position reachable. Come back to previous position" << endl;
		    cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
		    count = count + 1;
		    cout << "Graph dimension : " << graph2.size() << endl;
		    
		} else {
		    

		    cout << "-----------------------------------------------------------------"<<endl;
		    cout << "I came back to the original position since i don't have any other candidate position"<< endl;
		    cout << "Total cell visited :" << numConfiguration <<endl;
		    cout << "FINAL: Map not completely explored!" << endl;
		    cout << "-----------------------------------------------------------------"<<endl;
		    exit(0);
		}
	    
		sensedCells = newSensedCells;
		
	    }else{
		
		
		// need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
		list<Pose> frontiers;
		vector<pair<long,long> >::iterator it =candidatePosition.begin();
		for(it; it != candidatePosition.end(); it++){
		    Pose p1 = Pose((*it).first,(*it).second,0 ,range,FOV);
		    Pose p2 = Pose((*it).first,(*it).second,180,range,FOV);
		    Pose p3 = Pose((*it).first,(*it).second,90,range,FOV);
		    Pose p4 = Pose((*it).first,(*it).second,270,range,FOV);
		    frontiers.push_back(p1);
		    frontiers.push_back(p2);
		    frontiers.push_back(p3);
		    frontiers.push_back(p4);
		}
		
		unexploredFrontiers = frontiers;
		
		cout << "Graph dimension : " << graph2.size() << endl;
		cout << "Candidate position: " << candidatePosition.size() << endl;
		cout <<"Frontiers: "<<  frontiers.size() << endl;
		EvaluationRecords *record = function.evaluateFrontiers(frontiers,map,threshold);
		//cout << "Record: " << record->size() << endl;
		cout << "Evaluation Record obtained" << endl;
		nearCandidates = record->getFrontiers();
		
		if(record->size() != 0){
		    
		    //NOTE: TAKE THIS BRANCH IF THERE ARE CANDIDATE POSITION
		    
		    //set the previous pose equal to the actual one(actually represented by target)
		    previous = target;
		    std::pair<Pose,double> result = function.selectNewPose(record);
		    target = result.first;
		    if (contains(tabuList,target) == false){
			count = count + 1;
			numConfiguration++;
			history.push_back(function.getEncodedKey(target,1));
			tabuList.push_back(target);
			cleanPossibleDestination2(nearCandidates,target);
			std::pair<string,list<Pose>> pair = make_pair(actualPose,nearCandidates);
			graph2.push_back(pair);
			cout << "Graph dimension : " << graph2.size() << endl;
		    }else{
			if(graph2.size() == 0 ) break;file:///home/pulver/projects/mcdm_online_exploration_ros/Criteria/mcdmweightreader.cppfile:///home/pulver/projects/mcdm_online_exploration_ros/mcdmfunction.cpp
			if(graph2.at(graph2.size()-1).second.size() != 0){
			    cout << "[BT1 - Tabulist]There are visible cells but the selected one is already explored!Come back to second best position from the previous position"<< endl;
			    cleanPossibleDestination2(nearCandidates,target);
			    record = function.evaluateFrontiers(nearCandidates,map,threshold);
			    if(record->size() != 0){
				std::pair<Pose,double> result = function.selectNewPose(record);
				target = result.first;
				tabuList.push_back(target);
				
				history.push_back(function.getEncodedKey(target,2));
				count = count + 1;
				cout << "Graph dimension : " << graph2.size() << endl;
				btMode = true;
			    }else{
				if(graph2.size() == 0 ) break;
				string targetString = graph2.at(graph2.size()-1).first;
				graph2.pop_back();
				target = record->getPoseFromEncoding(targetString);
			    }
			}else{
			    graph2.pop_back();
			    string targetString = graph2.at(graph2.size()-1).first;
			    target = record->getPoseFromEncoding(targetString);
			    history.push_back(function.getEncodedKey(target,2));
			    cout << "[BT2 - Tabulist]There are visible cells but the selected one is already explored!Come back to two position ago"<< endl;
			    cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
			    count = count + 1;
			    cout << "Graph dimension : " << graph2.size() << endl;
			}
			
		    }
		}else {  
			
			//NOTE: TAKE THIS BRANCH IF THERE ARE NO CANDIDATE POSITIONS
		    
			if(graph2.size() == 0 ) break;
			
			
			//select as new target the previous one in the graph structure
			string targetString = graph2.at(graph2.size()-1).first;
			graph2.pop_back();
			
			target = record->getPoseFromEncoding(targetString);
			
			
			
			
			if(!target.isEqual(previous)){
			    previous = target;
			    cout << "[BT3]There are no visible cells so come back to previous position in the graph structure" << endl;
			    history.push_back(function.getEncodedKey(target,2));
			    cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
			    count = count + 1;
			    cout << "Graph dimension : " << graph2.size() << endl;
			    
			}else {
			    
			    if(graph2.size() == 0 ) {
				cout << "[BT4]No other possibilities to do backtracking on previous positions" << endl;
				break;
			    }
			    string targetString = graph2.at(graph2.size()-1).first;
			    graph2.pop_back();
			    
			    target = record->getPoseFromEncoding(targetString);
			    previous = target;
			    cout << "[BT5]There are no visible cells so come back to previous position" << endl;
			    cout << "[BT5]Cell already explored!Come back to previous position"<< endl;
			    history.push_back(function.getEncodedKey(target,2));
			    cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
			    count = count + 1;
			    cout << "Graph dimension : " << graph2.size() << endl;
			}
		
		}
	
		
		
		//NOTE: not requested for testing purpose
		//usleep(microseconds);
		sensedCells = newSensedCells;
		frontiers.clear();
		candidatePosition.clear();
		delete record;
	    }
	    
	}else{
	    
	    //NOTE: BT MODE
	    long x = target.getX();
	    long y = target.getY();
	    int orientation = target.getOrientation();
	    int range = target.getRange();
	    double FOV = target.getFOV();
	    string actualPose = function.getEncodedKey(target,0);
	    map.setCurrentPose(target);
	    travelledDistance = travelledDistance + target.getDistance(previous);
	    string encoding = to_string(target.getX()) + to_string(target.getY());
	    visitedCell.emplace(encoding,0);
	    
	    
	    
	    cout << "-----------------------------------------------------------------"<<endl;
	    cout << "|||| BT MODE ||||"<< endl;
	    cout << "Round : " << count<< endl;
	    newSensedCells = sensedCells + ray.getInformationGain(map,x,y,orientation,FOV,range);
	    cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;

	    EvaluationRecords *record = function.evaluateFrontiers(nearCandidates,map,threshold);
	    cout << "Evaluation Record obtained" << endl;
	    cout << "nearCandidates dimensions before choosing : " << nearCandidates.size() << endl;
	    if(record->size() != 0){
		
		//NOTE: TAKE THIS BRANCH IF THERE ARE CANDIDATE POSITION
		//set the previous pose equal to the actual one(actually represented by target)
		previous = target;
		cleanPossibleDestination2(nearCandidates,target);
		std::pair<Pose,double> result = function.selectNewPose(record);
		target = result.first;
		if (contains(tabuList,target) == false){
		    count = count + 1;
		    numConfiguration++;
		    history.push_back(function.getEncodedKey(target,1));
		    //cout << "Graph dimension : " << graph2.size() << endl;
		    tabuList.push_back(target);
		    cleanPossibleDestination2(nearCandidates,target);
		    std::pair<string,list<Pose>> pair = make_pair(actualPose,nearCandidates);
		    btMode = false;
		    cout << "[BT-MODE4] Go back to previous positions in the graph" << endl;
		}else{
		    if(nearCandidates.size() != 0){
			cout << "[BT-MODE1]Already visited" << endl;
			count = count + 1;
			EvaluationRecords *record = function.evaluateFrontiers(nearCandidates,map,threshold);
			previous = target;
			std::pair<Pose,double> result = function.selectNewPose(record);
			target = result.first;
			tabuList.push_back(target);
			cout << "nearCandidates dimensions after choosing : " << nearCandidates.size() << endl;

		    }else{
			cout << "[BT-MODE2] Go back to previous positions in the graph" << endl;
			string targetString = graph2.at(graph2.size()-1).first;
			graph2.pop_back();
			target = record->getPoseFromEncoding(targetString);
			previous = target;
			history.push_back(function.getEncodedKey(target,2));
			cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
			count = count + 1;
			btMode = false;
			
		    }
		}
	    }else{
		string targetString = graph2.at(graph2.size()-1).first;
		graph2.pop_back();
		target = record->getPoseFromEncoding(targetString);
		previous = target;
		history.push_back(function.getEncodedKey(target,2));
		cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
		count = count + 1;
		btMode = false;
		cout << "[BT-MODE3] Go back to previous positions in the graph" << endl;
	    }
	    delete record;
	}
    }
    
    map.drawVisitedCells(visitedCell,resolution);
    map.printVisitedCells(history);
  
   
    //OLD METHOD
    
    if (sensedCells >= precision * totalFreeCells ){
	cout << "-----------------------------------------------------------------"<<endl;
	cout << "Total cell visited :" << numConfiguration <<endl;
	cout << "Total travelled distance (cells): " << travelledDistance << endl;
	cout << "Total number of turning: " << numOfTurning << endl;
	cout << "FINAL: MAP EXPLORED!" << endl;
	cout << "-----------------------------------------------------------------"<<endl;
    }else{
	cout << "-----------------------------------------------------------------"<<endl;
	cout << "I came back to the original position since i don't have any other candidate position"<< endl;
	cout << "Total cell visited :" << numConfiguration <<endl;
	cout << "-----------------------------------------------------------------"<<endl;
	
    }
    
}

bool contains(std::list<Pose>& list, Pose& p){
    bool result = false;
    MCDMFunction function;
   
    std::list<Pose>::iterator findIter = std::find(list.begin(), list.end(), p);
    if (findIter != list.end()){
	//cout << "Found it: "<< function.getEncodedKey(p,0) <<endl;
	result = true;
    }

    return result;
}

void cleanPossibleDestination2(std::list< Pose >& possibleDestinations, Pose& p){
    MCDMFunction function;
    //cout<<"I remove "<< function.getEncodedKey(p,0) << endl;
    //cout << possibleDestinations->size() << endl;
    
    
    
    
    std::list<Pose>::iterator findIter = std::find(possibleDestinations.begin(), possibleDestinations.end(), p);
    if (findIter != possibleDestinations.end()){
	//cout << function.getEncodedKey(*findIter,0) << endl;
	possibleDestinations.erase(findIter);
    } else cout<< "not found" << endl;
    
   
    
}
