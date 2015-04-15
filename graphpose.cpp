/*
 * Copyright 2015 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "graphpose.h"
using namespace std;

graphPose::graphPose():
    graph(new vector<vector<Pose,Edge>>())
{
}

graphPose::~graphPose()
{

}

void graphPose::addPose(Pose& p,Pose &currentPose)
{
    Edge newEdge;
    newEdge.destination = p;
    newEdge.weight = currentPose.getDistance(p);
    // search in the graph the actual position of the robot and add the selected pose to its neighbors
    for(int i = 0; i != graph.size(); i++){
	if(graph[i][0].first == currentPose){
	    std::pair<Pose,Edge> pair(currentPose,newEdge);
	    graph[i].push_back(pair);
	}
    }
    // Add the new reached pose at the end of the graph
    pair<Pose,Edge> newPair (p, NULL);
    graph.push_back(newPair);
}

void graphPose::removePose(Pose& p)
{
    vector<vector<pair<Pose,Edge>>> :: iterator it ;
    
    //delete a pose from the graph
    for ( it = graph.begin(); it != graph.end(); it++){
	if((*it).at(0).first == p){
	graph[it].erase();
	}
    }
    
    //delete a pose from other poses in which appears as neighbor
    //scan the external vector
    for(int i = 0; i != graph.size(); i++){
	//scan each inner vector
	for (int j = 0; j < graph[i].size() ; j++){
	    if(graph[i][j].first ==p){
		graph[i].erase(j);
	    }
	}
    }
    
}


vector< pair< Pose, graphPose::Edge > > graphPose::getKnownDestination(Pose& p)
{
    for (int i = 0; i != graph.size(); i++){
	if(graph[i][0].first == p){
	return graph[i];
	}
    }
}


