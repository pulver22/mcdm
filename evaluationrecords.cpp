#include "evaluationrecords.h"
#include <functional>




EvaluationRecords::EvaluationRecords() :
    evaluations(new unordered_map<Frontier, double>())
{

}

EvaluationRecords::EvaluationRecords() :
    evaluations(new unordered_map<Frontier, double>())
{

}

EvaluationRecords::~EvaluationRecords()
{
    delete evaluations;
}

void EvaluationRecords::putEvaluation(Pose frontier, double value)
{
    if(evaluations->empty()){
	minValue = value;
	maxValue = value;
    }

    evaluations->insert(frontier, value);

    if(value >= maxValue)
	maxValue = value;
    if(value <= minValue)
	minValue = value;


}

double EvaluationRecords::getEvaluation(Frontier frontier)
{
    return evaluations[frontier];
}

bool EvaluationRecords::contains(Frontier frontier)
{
    unordered_map<Pose,double>::const_iterator got = evaluations->find(frontier);
    if (got == evaluations->end()){
	return false;
    }else return true;
}

int EvaluationRecords::size()
{
    return evaluations->size();
}



vector<Pose> *EvaluationRecords::getFrontiers() 
{
    
    vector<Pose> list;
    list.reserve(evaluations->size());

    for(unordered_map<Pose,double>::iterator it = evaluations->begin(); it != evaluations->end(); it++ ) {
	list.push_back((*it).first);
	
    } 
    
    vector<Pose> *toRet = new vector<Pose>();

    for(vector<Pose>::iterator it = list.begin(); it != list.end(); it++){
	toRet->push_back(*it);
    }

    return toRet;
}


unordered_map<Pose, double> * EvaluationRecords::getEvaluations()
{
    return evaluations;
}

/*
unordered_map<uint, double> EvaluationRecords::getEvaluationsWithHashCode()
{
    unordered_map<uint, double> evaluationsRecords;
    std::hash<Pose> pose_hash;

    for(unordered_map<Pose,double>::iterator it = evaluations->begin(); it != evaluations->end(); it++){
	pair<uint,double> hashPair (pose_hash((*it.).first),(*evaluations)[(*it).first]);
	evaluationsRecords.insert(hashPair);
    }
    return evaluationsRecords;
}*/


void EvaluationRecords::removeFrontier(Pose frontier)
{        
    for(unordered_map<Pose,double>::iterator it = evaluations->begin(); it != evaluations->end(); it++){
	if( (*it).first == frontier){
	    evaluations->erase(frontier);
	}
    }
}

void EvaluationRecords::normalize(){
    vector<Pose> list ;
    list.reserve(evaluations->size());

    for(unordered_map<Pose,double>::iterator it = evaluations->begin(); it != evaluations->end(); it++ ) {
	list.push_back((*it).first);
    } 
    
    for(unordered_map<Pose,double>::iterator it = list.begin(); it != list.end(); it++ ){
	for(unordered_map<Pose,double>::iterator it2 = evaluations->begin(); it2 != evaluations->end(); it2++){
	    if ((*it) == (*it2).first){
		double value = (*it2).second;
		value = (value - minValue)/(maxValue-minValue);
		evaluations->erase((*it2).first);
		pair<Pose,double> newPair (*it, value);
		evaluations->insert(newPair);
	     }
	 }
    }


}


