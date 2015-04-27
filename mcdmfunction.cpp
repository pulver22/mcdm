#include "mcdmfunction.h"
#include "explorationconstants.h"
#include "Criteria/criterion.h"
#include "Criteria/criteriaName.h"
#include "Criteria/traveldistancecriterion.h"
#include "Criteria/informationgaincriterion.h"
#include "Criteria/sensingtimecriterion.h"
#include "Criteria/mcdmweightreader.h"
#include "Criteria/criterioncomparator.h"
#include <string>
#include <iostream>
#include <stdlib.h>
#include <algorithm>


using namespace std;
using namespace import_map;
/* create a list of criteria with name and <encoded_name,weight> pair after reading that from a file
 */
MCDMFunction::MCDMFunction() //:
     //criteria(new unordered_map<string, Criterion* >())
     //activeCriteria(new vector<Criterion >() )
    
{
    /*read the weight from somewhere
    MCDMWeightReader weightReader;
    matrix = weightReader.parseFile();
    */
    
    // Initialization ad-hoc
    
    MCDMWeightReader reader;
    //cout << "test" << endl;
    matrix = reader.getMatrix();
    //cout << "test2" << endl;

    // get the list of all criteria to be considered
    list<string> listCriteria = matrix->getKnownCriteria();
    for (list< string >::iterator it = listCriteria.begin(); it != listCriteria.end(); ++it){
	string name = *it;
	// retrieve the weight of the criterion using the encoded version of the name
	double weight = matrix->getWeight(matrix->getNameEncoding(name));
	Criterion *c = createCriterion(name, weight);
	if(c != NULL)
	    criteria.emplace(name, c);
    }

}

MCDMFunction::~MCDMFunction()
{	/*
    //delete matrix;
    for (int i= criteria.size()-1; i >=0; i--){
	criteria.erase(i);
    }
    delete criteria;
    */

}


Criterion * MCDMFunction::createCriterion(string name, double weight)
{
    Criterion *toRet = NULL;
    if(name == string(SENSING_TIME)){
	SensingTimeCriterion toRet =  SensingTimeCriterion(weight);
    } else if (name == string(INFORMATION_GAIN)) {
	InformationGainCriterion toRet =  InformationGainCriterion(weight);
    } else if (name == string(TRAVEL_DISTANCE)){
	TravelDistanceCriterion toRet =  TravelDistanceCriterion(weight);
    }
    return toRet;
}


double MCDMFunction::evaluateFrontier( Pose p,  import_map::Map &map)
{
    //Should keep the ordering of the criteria and the weight of each criteria combinations
   for (vector<Criterion *>::iterator it = activeCriteria.begin(); it != activeCriteria.end(); it++){
       Criterion *c = *it;

       c->evaluate(p,map);
   }
   
    //for loop, over the criteria, to compute the utility of the frontier.

    return 0.0;
}



EvaluationRecords* MCDMFunction::evaluateFrontiers( std::list< Pose >& frontiers,  Map& map)
{         
    myMutex.lock();
   
    //Clean the last evaluation
    //         lprint << "clean evaluations" << endl;
    // ATTENTION : to be fixed -> you need to clear criteria[ i ].Criterion ...
    //NOTE: probably working
    unordered_map<string,Criterion *>::iterator it;
    for(it = criteria.begin(); it != criteria.end(); it++){
	std::pair<string,Criterion*> pair = *it;
	(criteria.at(pair.first))->clean();
    }
    
    /*
    //Get the list of activeCriteria
    if(activeCriteria.size() == 0){
	activeCriteria = new vector<Criterion >();
    }
    */
    
    // listActiveCriteria contains the name of the criteria while "criteria struct" contain the pairs <name, criterion>
    vector<string> listActiveCriteria = matrix->getActiveCriteria();
    for(vector<string>::iterator it = listActiveCriteria.begin(); it != listActiveCriteria.end(); it++){
	activeCriteria.push_back(criteria[*it]);
    }
    
    //ldbg << "number of active criteria: "<<activeCriteria->size() << endl;

  
    //Evaluate the frontiers
    list<Pose>::iterator it2 ;
    for (it2 = frontiers.begin(); it2 != frontiers.end(); it2++){
	Pose f = *it2;
	double value = 0.0;
	value = evaluateFrontier(f, map);
    }
    
    
    //Normalize the values
    for(vector<Criterion *>::iterator it = activeCriteria.begin(); it != activeCriteria.end(); ++it){
	(*it)->normalize();
    }
    
    
    //Create the EvaluationRecords
    //         lprint << "#number of frontier to evaluate: "<<frontiers.size()<<endl;
    EvaluationRecords *toRet = new EvaluationRecords();
    
    // analyze every single frontier f, and add in the evaluationRecords <frontier, evaluation>
    for(list<Pose>::iterator i=frontiers.begin(); i!=frontiers.end(); i++){

	Pose f = *i;
	// order criteria depending on the considered frontier
	//qsort(&activeCriteria,activeCriteria.size(),sizeof(Criterion),CriterionComparator(f));
	sort(activeCriteria.begin(),activeCriteria.end(),CriterionComparator(f));
	//apply the choquet integral
	Criterion *lastCrit = NULL;
	double finalValue = 0.0;
	for(vector<Criterion *>::iterator i = activeCriteria.begin(); i != activeCriteria.end(); i++){
	    Criterion *c = NULL ;
	    double weight = 0.0;
	    //Get the list of criterion whose evaluation is >= than the one's considered
	    list<string> names;
	   for(vector<Criterion *>::iterator j = i+1; j != activeCriteria.end(); j++){
	       //CHECK IF THE ITERATOR RETURN THE COUPLE <STRING,CRITERION>
	       Criterion *next = (*j);
	       names.push_back(next->getName());
	   }
	    weight = matrix->getWeight(names);
	
//               lprint << "#"<<names << " - w = " << weight << " - ";
//               lprint << names <<" with weight "<<weight<<endl;
	    if(i==activeCriteria.begin()){
		c = (*i);
		finalValue += c->getEvaluation(f) * weight;
//                         lprint << "#crit "<<c->getName()<<" - eval = "<<c->getEvaluation(f) << endl;
	    } else {
		c = (*i);
		double tmpValue = c->getEvaluation(f)-lastCrit->getEvaluation(f);
//                         lprint << "#crit "<<c->getName()<<" - eval = "<<c->getEvaluation(f) << endl;
		finalValue += tmpValue*weight;
	    }
	    lastCrit = c;
	}
	cout << f.getX() <<";"<<f.getY();
	cout <<";"<<finalValue << ";"<< endl;
	toRet->putEvaluation(f, finalValue);
    }
//         }
    cout << endl;
    
    //delete activeCriteria;
    //activeCriteria = NULL;
    myMutex.unlock();
    return toRet;
}

Pose MCDMFunction::selectNewPose(EvaluationRecords &evaluationRecords)
{
    Pose newTarget;
    double value;
    unordered_map<string,double> evaluation = evaluationRecords.getEvaluations();
    for(unordered_map<string,double>::iterator it = evaluation.begin(); it != evaluation.end(); it++){
	string tmp = (*it).first;
	Pose p = evaluationRecords.getPoseFromEncoding(tmp);
	if(value == 0){
	    newTarget = p ;
	}else if(value < (*it).second){
		newTarget = p;
	    } else continue;
    }
    
    return newTarget;
}

