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

#ifndef TRAVELDISTANCECRITERION_H
#define TRAVELDISTANCECRITERION_H

#include "criterion.h"
#include "pose.h"
#include "map.h"
#include "PathFinding/astar.h"
//using namespace import_map;
class TravelDistanceCriterion :  public Criterion
{
    public:
	TravelDistanceCriterion(double weight);
	~TravelDistanceCriterion();
	double evaluate( Pose &p, dummy::Map *map, RFID_tools *rfid_tools, double *batteryTime);
	//only for testing purpose
	//void insertEvaluation(Pose &p, double value);
	protected:
	Astar astar;
    double distance = 0.0;
};

#endif // TRAVELDISTANCECRITERION_H
