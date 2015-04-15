#include "mcdmfunction.h"
#include "explorationconstants.h"
#include "Criteria//criteriaName.h"
#include "Criteria/traveldistancecriterion.h"
#include "Criteria/informationgaincriterion.h"
#include "Criteria/sensingtimecriterion.h"
//#include "Criteria/mcdmweightreader.h"
#include "Criteria/criterioncomparator.h"
#include <string>
#include <iostream>



/* create a list of criteria with name and <encoded_name,weight> pair after reading that from a file
 */
MCDMFunction::MCDMFunction() :
     criteria(new unordered_map<string, Criterion *>()),activeCriteria(NULL)
    
{
    /*read the weight from somewhere
    MCDMWeightReader weightReader;
    matrix = weightReader.parseFile();
    */
    
    // Initialization ad-hoc
    WeightMatrix matrix = new WeightMatrix(3);
    matrix.insertSingleCriterion(INFORMATION_GAIN,0.5,true);
    matrix.insertSingleCriterion(TRAVEL_DISTANCE,0.2, true);
    matrix.insertSingleCriterion(SENSING_TIME,0.3, true);
    list<string> list1 (INFORMATION_GAIN,TRAVEL_DISTANCE);
    list<string> list2 (INFORMATION_GAIN,SENSING_TIME);
    list<string> list3 (SENSING_TIME,TRAVEL_DISTANCE);
    list<string> list4 (SENSING_TIME,TRAVEL_DISTANCE,INFORMATION_GAIN);
    matrix.insertCombinationWeight(list1,0.8);
    matrix.insertCombinationWeight(list2,0.6);
    matrix.insertCombinationWeight(list3,0.6);
    matrix.insertCombinationWeight(list4,1);

    // get the list of all criteria to be considered
    list<string> listCriteria = matrix.getKnownCriteria();
    for (list< string >::iterator it = listCriteria.begin(); it != listCriteria.end(); ++it){
	string name = *it;
	// retrieve the weight of the criterion using the encoded version of the name
	double weight = matrix.getWeight(matrix.getNameEncoding(name));
	Criterion *c = createCriterion(name, weight);
	if(c != NULL)
	    criteria->insert(name, c);
    }
    

    //        QHash<QString, double> * configuration = weightReader.parseFile();
    //        foreach(QString crit, configuration->keys()){
    //            if(crit.contains("_")){
    //                jointCriteria.insert(crit, configuration->value(crit));
    //            } else {
    //                Criterion *criterion = createCriterion(crit, configuration->value(crit));
    //                if(criterion!=NULL){
    //                    criteria->append(criterion);
    //                }
    //            }
    //        }
    //        delete configuration;

}

MCDMFunction::~MCDMFunction()
{
    delete matrix;
    for (int i= criteria->size()-1; i >=0; i--){
	criteria->erase(i);
    }
    delete criteria;

}


Criterion * MCDMFunction::createCriterion(string name, double weight)
{
    Criterion *toRet = NULL;
    if(name == string(SENSING_TIME)){
	toRet = new SensingTimeCriterion(weight);
    } else if (name == string(INFORMATION_GAIN)) {
	toRet = new InformationGainCriterion(weight);
    } else if (name == string(TRAVEL_DISTANCE)){
	toRet = new TravelDistanceCriterion(weight);
    }
    return toRet;
}


double MCDMFunction::evaluateFrontier( const Pose *p, const Map &map)
{
    //Should keep the ordering of the criteria and the weight of each criteria combinations
   for (vector<Criterion *>::iterator it = *activeCriteria.begin(); it != activeCriteria.end(); it++){
       Criterion *c = *it;
       c->evaluate(p,map);
   }
   
    //for loop, over the criteria, to compute the utility of the frontier.

    return 0.0;
}



EvaluationRecords* MCDMFunction::evaluateFrontiers(const std::list< Pose* >& frontiers, const Map& map)
{         
    myMutex.lock();
   
    //Clean the last evaluation
    //         lprint << "clean evaluations" << endl;
    for(int i=0; i< criteria->size(); i++){
	criteria->clear(i);
    }

    //Get the list of activeCriteria
    if(activeCriteria == NULL){
	activeCriteria = new vector<Criterion *>();
    }
    vector<string> listActiveCriteria = matrix->getActiveCriteria();
    for(vector<string>::iterator it = listActiveCriteria.begin(); it != listActiveCriteria.end(); it++){
	activeCriteria.push_back(criteria[*it]);
    }
    
    //ldbg << "number of active criteria: "<<activeCriteria->size() << endl;

  
    //Evaluate the frontiers
    for (int i=0; i<frontiers.size(); i++){
	//Check if the frontier is reachable.
	Pose *f = frontiers[i];
//             Point posePoint(f->centroid().x(), f->centroid().y());
//             bool evaluate = PathPlanner::frontierFound(map.frontiers(), posePoint);
	double value = 0.0;
//             if(evaluate){
	    //The frontier is reachable => evaluate them
//             lprint << f->centroid().x() <<";"<<f->centroid().y()<<endl;
	value = evaluateFrontier(f, map);
//             } else {
//                 //The frontier is not reachable => set the worst value
//                 foreach(Criterion *c, *activeCriteria){
//                     c->setWorstValue(f);
//                 }
//             }
    }
    
    //Normalize the values
    for(vector<Criterion *>::iterator it = activeCriteria.begin(); it != activeCriteria.end(); ++it){
	(*it).normalize();
    }
    
    
    //Create the EvaluationRecords
    //         lprint << "#number of frontier to evaluate: "<<frontiers.size()<<endl;
    EvaluationRecords *toRet = new EvaluationRecords();
    
    for(list<Pose>::iterator i=frontiers.begin(); i!=frontiers.end(); i++){

	Pose *f = *i;
	qsort(activeCriteria,activeCriteria.size(),sizeof(Criterion),CriterionComparator(f));
	//qSort(activeCriteria->begin(), activeCriteria->end(), CriterionComparator(f));
	
	//apply the choquet integral
	Criterion *lastCrit = NULL;
	double finalValue = 0.0;
	for(vector<Criterion *>::iterator i = activeCriteria.begin(); i != activeCriteria.end(); i++){
	    Criterion *c = NULL;
	    double weight = 0.0;
	    //Get the list of criterion that are >= than the one considered
	    list<string> names;
	   for(vector<Criterion *>::iterator j = i+1; j != activeCriteria.end(); j++){
	       //CHECK IF THE ITERATOR RETURN THE COUPLE <STRING,CRITERION>
	       Criterion *c = (*j);
	       names.push_back(c->getName());
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
	cout << f->getX() <<";"<<f->getY();
	cout <<";"<<finalValue << ";"<< endl;
	toRet->putEvaluation(*f, finalValue);
    }
//         }
    cout << endl;
    
    delete activeCriteria;
    activeCriteria = NULL;
    myMutex.unlock();
    return toRet;
}

Pose MCDMFunction::selectNewPose(const EvaluationRecords &evaluationRecords)
{
    pair <Pose,double> newTarget;
    unordered_map<Pose,double> evaluation = evaluationRecords.getEvaluations();
    for(unordered_map<Pose,double>::iterator it = evaluation.begin(); it != evaluation.end(); it++){
	if(newTarget.second == NULL){
	    newTarget = *it ;
	}else if(newTarget.second < (*it).second){
		newTarget = *it;
	    } else continue;
    }
    
    return newTarget.first;
}


