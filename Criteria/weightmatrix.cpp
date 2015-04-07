#include "weightmatrix.h"



WeightMatrix::WeightMatrix(int numOfCriteria) :
    mapping(new map<string, string>()),
    weights(new list<map<string, double> *>()), 
    lastInsertedCriteria(64), 
    mutex(new mutex())
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
    mapping->clear();
    delete mapping;
}

void WeightMatrix::insertSingleCriterion(string name, double weight)
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
    mapping->insert(name, code);
    //insert the weight of the single criterion in the first row of the weight matrix
    if (weights->size() > 0)
	weights->pop_front(code,weight);
    mutex->unlock();
}

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



const string WeightMatrix::getNameEncoding(string name) const
{
    mutex->lock();
    const QString toRet = mapping->value(name);
    mutex->unlock();
    return toRet;
}

void WeightMatrix::insertCombinationWeight(list<string> criteriaNames, double weight)
{

    insertCombinationWeight(computeNamesEncoding(criteriaNames), weight);

}

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
    weights->at(card-1)->insert(encoding, weight);
    mutex->unlock();
}

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
    double toRet = weights->at(card-1)->value(encoding);
    mutex->unlock();
    return toRet;
}

string WeightMatrix::computeNamesEncoding(list<string> criteriaNames) const
{
    //ldbg << "Criteria Names: " << criteriaNames << endl;
    mutex->lock();
    if (criteriaNames.empty())
            return "";
    list<string> enc;
    for(int i=0; i<criteriaNames.size(); i++){
        enc.append(mapping->value(criteriaNames.at(i)));
    }
//    ldbg << "Encode before sorting: " << enc << endl;
    qSort(enc);
//    ldbg << "Encode after sorting: " << enc << endl;
    QString toRet;
    for(int i=0; i<enc.size(); i++){
        toRet.append(enc.at(i));
    }
    mutex->unlock();
    //ldbg << "Encoding " << toRet << endl;
    return toRet;
}



