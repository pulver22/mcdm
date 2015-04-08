#include "mcdmfunction.h"
#include "explorationconstants.h"
#include "Criteria//criteriaName.h"
#include "Criteria/traveldistancecriterion.h"
#include "Criteria/informationgaincriterion.h"
#include "Criteria/sensingtimecriterion.h"
#include "Criteria//mcdmweightreader.h"
#include "Criteria/criterioncomparator.h"
#include <string>



MCDMFunction::MCDMFunction() :
     criteria(new unordered_map<string, Criterion *>())
    
{
    //read the weight from somewhere
    MCDMWeightReader weightReader;
    matrix = weightReader.parseFile();
    list<string> listCriteria = matrix->getKnownCriteria();
    list< string >::iterator l_front = listCriteria.begin();
    for (l_front; l_front != listCriteria.end(); ++l_front){
	string name = l_front;
	double weight = matrix->getWeight(matrix->getNameEncoding(name));
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


Criterion * MCDMFunction::createCriterion(QString name, double weight)
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

//ATTENZIONE: VA SISTEMATO PERCHE' AD .at() BISOGNA PASSARE LA CHIAVE E NON L'ITERATORE
double MCDMFunction::evaluateFrontier( const Pose *p, const Map &map)
{
    //Should keep the ordering of the criteria and the weight of each criteria combinations
   for (int i = 0; i < criteria->size(); i++){
       Criterion *c = criteria->at(i);
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
    foreach(Criterion *c, *activeCriteria){
//             lprint << "#"<<c->getName() <<": "<<c->evaluationSize() <<endl;
	c->normalize();
    }
    semCrit.normalize();
    //Create the EvaluationRecords
//         lprint << "#number of frontier to evaluate: "<<frontiers.size()<<endl;
    EvaluationRecords *toRet = new EvaluationRecords();
    for(int i=0; i<frontiers.size(); i++){

	Frontier *f = frontiers[i];
	qSort(activeCriteria->begin(), activeCriteria->end(), CriterionComparator(f));
	//apply the choquet integral

	Criterion *lastCrit = NULL;
	double finalValue = 0.0;
	double semEval = -1.0;
//             lprint << i << ") ";
//             Point myPoint(map.lastRobotPose(Config::robotID)->x(), map.lastRobotPose(Config::robotID)->y());
//             Point centroid = f->centroid();
//             if(myPoint.distance(centroid) < LASER_RANGE){
	    for(int i=0; i<activeCriteria->length(); i++){
		//ldbg << "MCDM : value of i "<<i<<endl;
		//ldbg << "MCDM : size of active criteria "<<activeCriteria->length()<<endl;
		Criterion *c = NULL;
		double weight = 0.0;
		//Get the list of criterion that are >= than the one considered
		QList<QString> names;
		for(int j=i ; j<activeCriteria->length(); j++){
		    Criterion *next = activeCriteria->at(j);
		    names.append(next->getName());
		}

		weight = matrix->getWeight(names);
		//lprint << "#"<<names << " - w = " << weight << " - ";
//                     lprint << names <<" with weight "<<weight<<endl;
		if(i==0){
		    c = activeCriteria->first();
		    finalValue += c->getEvaluation(f) * weight;
		    if (c->getName() == QString(SEMANTIC)){
			semEval = c->getEvaluation(f);
		    }
//                         lprint << "#crit "<<c->getName()<<" - eval = "<<c->getEvaluation(f) << endl;
		} else {
		    c = activeCriteria->at(i);
		    double tmpValue = c->getEvaluation(f)-lastCrit->getEvaluation(f);
//                         lprint << "#crit "<<c->getName()<<" - eval = "<<c->getEvaluation(f) << endl;
		    finalValue += tmpValue*weight;
		    if (c->getName() == QString(SEMANTIC)){
			semEval = c->getEvaluation(f);
		    }
		}
		lastCrit = c;
	    }
	    if(semEval < 0.0){
		semEval = semCrit.getEvaluation(f);
	    }
	    lprint << QString::number(f->centroid().x(), 'f', 2) <<";"<<QString::number(f->centroid().y(), 'f', 2);
	    lprint <<";"<<QString::number(finalValue, 'f', 2) << ";"<< QString::number(semEval, 'f', 2) << endl;
	toRet->putEvaluation(*f, finalValue);
    }
//         }
    lprint << endl;
    delete activeCriteria;
    activeCriteria = NULL;
    myMutex.unlock();
    return toRet;
}

void MCDMFunction::onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command)
{
    matrix->changeCriteriaActivation(USER_DIRECTION, false);
    matrix->changeCriteriaActivation(USER_AREA, false);
    if(activate){
	bool expDir = command->exploreDirection();
	if(expDir){
	    matrix->changeCriteriaActivation(USER_DIRECTION, true);
	    UserDirectionCriterion *c = (UserDirectionCriterion *)criteria->value(USER_DIRECTION);
	    c->setAlphaX(command->getX());
	    c->setAlphaY(command->getY());
	    ldbg << "userDirection with weights: x = " << command->getX() <<", "<<command->getY()<<endl;
	} else {
	    matrix->changeCriteriaActivation(USER_AREA, true);
	    UserAreaCriterion *c = (UserAreaCriterion *)criteria->value(USER_AREA);
	    Point p(command->getX(), command->getY());
	    c->setPoint(p);
	    ldbg << "userArea with point: <" << command->getX() <<", "<<command->getY()<< ">" <<endl;
	}
    }
}

void MCDMFunction::setMultimap(Multimap *multiMap)
{
    this->multimap = multiMap;

    foreach(Criterion *c, criteria->values()){
	ldbg << "Set the global map to the criterion "<<c->getName()<<endl;
	c->setMultimap(multiMap);
    }

}

