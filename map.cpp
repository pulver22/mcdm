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

#include "map.h"
#include <vector>

using namespace std;

Map::Map()
{
}

Map::Map(int height, int width, float resolution, vector< vector<int>> &map2D):
	    height(height), width(width),resolution(resolution), map2D(map2D)
{

}

Map::~Map()
{

}

int Map:: getWidth(){
    return width;
}

int Map:: getHeight(){
    return height;
}

float Map:: getResolution(){
    return resolution;
}

vector<vector<int>> Map::getMap2D(){
    return map2D;
}

Pose Map::getRobotPosition(){
    // do something, maybe with the planner...
}




