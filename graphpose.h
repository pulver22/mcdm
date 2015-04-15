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

#ifndef GRAPHPOSE_H
#define GRAPHPOSE_H
#include "pose.h"
#include <vector>
#include <utility>

using namespace std;

class graphPose
{
    
struct Edge
{
    Pose destination;
    double weight;
};

public:
    graphPose();
    ~graphPose();
    //add the p pose in the graph of known pose as new state but also as reachable from a know position
    void addPose(Pose &p, Pose &currentPose);
    //remove one pose from the graph
    void removePose(Pose &p);
    //return the vector contening the pose (and the travel cost express as distance) reachable from the one passed as variable
    std::vector< std::pair< Pose,Edge > > getKnownDestination(Pose& p);
    
private:
  Edge edge;
  vector<vector<pair<Pose,Edge>>> graph;

    
};



#endif // GRAPHPOSE_H
