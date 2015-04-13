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

#include "criterion.h"


Criterion::Criterion()
{

}

Criterion:: Criterion(const string &name, double weight, bool highGood):
    name(name), weight(weight),highGood(highGood)
{

}

Criterion::~Criterion()
{

}

void Criterion::insertEvaluation(Pose &p, double value)
{
//    if(evaluation.contains(point))
//        lprint << "#repeated frontier!!!" << endl;
    evaluation.insert(p, value);
    if(value >= maxValue)
        maxValue = value;
    if(value <= minValue)
        minValue = value;
}

void Criterion::clean()
{
//    for(QHash<SLAM::Geometry::Frontier *, double>::iterator it = evaluation.begin(); it!=evaluation.end(); it++){
//        delete it.key();
//    }
    evaluation.clear();
}

void Criterion::normalize()
{
    if(highGood)
        normalizeHighGood();
    else
        normalizeLowGood();
}

void Criterion::normalizeHighGood()
{
    unordered_map<const Pose *, double> temp;
    for (auto& it : evaluation ){
	pair p =  *it;
       double value =p.second;
       value = (value-minValue)/(maxValue-minValue);
       temp.emplace(p.first, value);
   }
    evaluation = temp;
}

void Criterion::normalizeLowGood()
{
    unordered_map<const Pose *, double> temp;
    for (auto& it : evaluation ){
	pair p =  *it;
        double value =p.second;
        value = (maxValue-value)/(maxValue-minValue);
        temp.emplace(p.first, value);
    }
    evaluation = temp;
}


double Criterion::getEvaluation(Pose p) const
{
    return evaluation[p];
}


const string& Criterion::getName() const
{
    return name;
}

double Criterion::getWeight() const
{
    return weight;
}

void Criterion::setName(const string& name)
{
    this->name = name;
}

void Criterion::setWeight(double weight)
{
    this->weight = weight;
}

