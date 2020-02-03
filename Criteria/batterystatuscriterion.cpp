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

#include "Criteria/batterystatuscriterion.h"
#include "Criteria/criteriaName.h"
#include <iostream>




BatteryStatusCriterion::BatteryStatusCriterion(double weight)
	: Criterion(BATTERY_STATUS, weight, true)
{
}


BatteryStatusCriterion::~BatteryStatusCriterion()
{

}

double BatteryStatusCriterion::evaluate( Pose &p, dummy::Map *map, RadarModel *rm, double *batteryTime)
{
    Pose robotPosition = map->getRobotPosition();
    //double distance = robotPosition.getDistance(p);
    string path = this->astar.pathFind(robotPosition.getX(),robotPosition.getY(),p.getX(),p.getY(), map);
    distance = this->astar.lengthPath(path);
    numOfTurning = this->astar.getNumberOfTurning ( path );
    translTime = distance / TRANSL_SPEED;
    rotTime = numOfTurning / ROT_SPEED;
    timeRequired = translTime + rotTime;
    remainingBattery = *batteryTime - timeRequired;
    // std::cout << "Distance: " << distance << endl;
    //cout << "alive after calling a*" << endl;
    Criterion::insertEvaluation(p, remainingBattery);
    return remainingBattery;
}

/*
void BatteryStatusCriterion::insertEvaluation(Pose& p, double value)
{
    cout << "alice" <<endl;
    insertEvaluation(p,value);
}
*/
