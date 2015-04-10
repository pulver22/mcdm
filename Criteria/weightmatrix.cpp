#include "weightmatrix.h"


/* mapping contains the criterion's name and his encoding
 * weights is a list (vector - for pratical purpose) of <encoding, weight> pairs */
WeightMatrix::WeightMatrix(int numOfCriteria) :
    mapping(new unordered_map<string, string>()),
    activeCriteria(new vector<string, bool>()), 
    weights(new vector<unordered_map<string, double> *>()), 
    lastInsertedCriteria(64), 
    mutex(new mutex()),
    numOfActiveCriteria(0)
{
    //Create a row in the weight matrix for each weight cardinality
    for(int i=0; i<numOfCriteria-1; i++){
        weights->push_back(new hash<string, double>());
    }

}

WeightMatrix::~WeightMatrix()
{
    delete mutex;
    for(int i=weights->size()-1; i>=0; i--){
        weights->erase(i);
    }
    delete weights;
    activeCriteria->clear();
    delete activeCriteria;
    mapping->clear();
    delete mapping;
}

/*insert the criterion's name and its encoding in mapping unordered_map(hast_table) and 
 * his encoding and his weight in weights vector */
void WeightMatrix::insertSingleCriterion(string name, double weight, bool active)
{
    mutex->lock();
    //increase the number of inserted criteria
    lastInsertedCriteria++;
    //encode the inserted criterion.
    //The coding of the single criteria is "A" for the first on,m "B" for the second one, "C" for the third...
    //ldbg << "Criterion Names: " << name << endl;
    string code((char)lastInsertedCriteria);
    //ldbg << "Criterion Code: " << code << endl;
    //insert the entry in the mapping table
    std::pair<string,string> pair (name,code); 
    mapping->insert(pair);
    std::pair<string,bool> pairActive (code, active);
    activeCriteria->push_back(pairActive);
    if(active)
	numOfActiveCriteria++;
    //insert the weight of the single criterion in the first row of the weight matrix
    if (weights->size() > 0)
	weights->emplace(0,code,weight);
	//weights->push_front(code,weight);
    mutex->unlock();
}

void WeightMatrix::changeCriteriaActivation(const string &name, bool active)
{
    mutex->lock();
    //Get the encoding of the criterion
    string enc = mapping[name];
    //Get the actual state of the criterion activation
    bool actualState;
    for(vector<string,bool>::iterator it = activeCriteria->begin(); it != activeCriteria->end(); it++){
	if((*it).first == enc)
	   actualState = (*it).second;
    }
    
    if(actualState == active){
        //if the state should not change, return
        mutex->unlock();
        return;
    }
    //If the state should change, override the past state of activation.
    activeCriteria->push_back(enc, active);
    if(active) //if the new state is a positive one, increase the number of active criteria
        numOfActiveCriteria++;
    else  //if the state is negative, decrease the number of active criteria.
        numOfActiveCriteria--;
    mutex->unlock();
}

/*
 */
double WeightMatrix::getWeight(list<string> criteriaNames) const
{

    string enc = computeNamesEncoding(criteriaNames);
    double w = getWeight(enc);
    if (w == 0){
        //no encoding saved. I must compute the weight by summing
        //up every single weight.

        for(int i=0; i<enc.length(); i++){
            string e(enc.at(i));
            w += getWeight(e);
        }
    }
    if(w>1) //weights must belong to [0,1].
        w = 1;

    return w;
}

int WeightMatrix::getNumOfActiveCriteria() const
{
    mutex->lock();
    int toRet = numOfActiveCriteria;
    mutex->unlock();
    return toRet;
}

/* return the encoding of the name searching for it in the mapping structure
 */
const string WeightMatrix::getNameEncoding(string name) const
{
    mutex->lock();   
    const string toRet = mapping[name];
    mutex->unlock();
    return toRet;
}

/*  given a list of criteria, call the proper function to encode the list in a single string and then insert
 * it with the provided weight
 */
void WeightMatrix::insertCombinationWeight(list<string> criteriaNames, double weight)
{

    insertCombinationWeight(computeNamesEncoding(criteriaNames), weight);

}

/*  insert in the weights structure the pair <enconding, weight> where enconding is a string taking care of more
 * than one criterion. 
 */
void WeightMatrix::insertCombinationWeight(const string &encoding, double weight)
{
    int card = encoding.length();
    mutex->lock();
    int mappingSize = mapping->size();
    mutex->unlock();

    if(card >= mappingSize)
        return;
    if(card <= 0)
        return;

    mutex->lock();
    weights->insert(card-1, std::pair<string,double>(encoding,weight));
    //weights->at(card-1)->emplace(encoding, weight);
    mutex->unlock();
}

/* given an encoding of a criterion, return the weight associated to that criterion
 */
double WeightMatrix::getWeight(const string &encoding) const
{
    //ldbg << "wights length = " << weights->length() << endl;
    int card = encoding.length();
    mutex->lock();
    int numActiveCrit = numOfActiveCriteria;
    mutex->unlock();
    if(card >= numActiveCrit)
        return 1;
    if(card <= 0)
        return 0;
    mutex->lock();
    double toRet = weights->at(card-1)[encoding];
    mutex->unlock();
    return toRet;
}

/* given a list of criteria names (could be only one), copy the list of the respective encoding in the enc list, 
 * then sort it and append every single encoded criterion in the toRet string
 */
string WeightMatrix::computeNamesEncoding(list<string> criteriaNames) const
{
    //ldbg << "Criteria Names: " << criteriaNames << endl;
    mutex->lock();
    if (criteriaNames.empty())
            return "";
    list<string> enc;
    list< string >::iterator l_front = criteriaNames.begin();
    for(l_front; l_front != criteriaNames.end(); ++l_front){
        enc.emplace(mapping[l_front]);
	
    }
//    ldbg << "Encode before sorting: " << enc << endl;
    enc.sort();
//    ldbg << "Encode after sorting: " << enc << endl;
    string toRet;
    for(l_front=enc.begin(); l_front != enc.end(); ++l_front){
        toRet.append(l_front);
    }
    mutex->unlock();
    //ldbg << "Encoding " << toRet << endl;
    return toRet;
}


vector<string> WeightMatrix::getActiveCriteria() const
{
    mutex->lock();
    vector<string> toRet;
    for(vector<string,bool>::iterator it =activeCriteria->begin(); it != activeCriteria->end(); it++){
	const string k = (*it).first;
	if((*it).second){
	    string toApp = mapping[k];
	    toRet.push_back(toApp);
	    
	}
    }
  
    mutex->unlock();
    return toRet;
}

/* return the list of all criteria considered
 */
list<string> WeightMatrix::getKnownCriteria() const
{
    list<string> toRet;
    mutex->lock();
    for (int i=0; i < mapping->size(); i++){
	toRet.emplace_back(mapping->at(i));
    }
    mutex->unlock();
    return toRet;
}



