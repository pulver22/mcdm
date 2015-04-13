#ifndef WEIGHTMATRIX_H
#define WEIGHTMATRIX_H

#include <functional>
#include <string>
#include <unordered_map>
#include <list>
#include <vector>
#include <iterator> 
#include <mutex>
#include <map>

using namespace std;
class WeightMatrix
{
public:
    WeightMatrix(int numOfCriteria);
    virtual ~WeightMatrix();

    void insertSingleCriterion(string name, double weight, bool active);
    const string getNameEncoding(string name) const;

    void insertCombinationWeight(list<string> criteriaNames, double weight);
    void insertCombinationWeight(const string &encoding, double weight);
    double getWeight(list<string> criteriaNames) const;
    double getWeight(const string &encoding) const;        
    string computeNamesEncoding(list<string> criteriaNames) const;
    list<string> getKnownCriteria() const;
    vector<string> getActiveCriteria() const;
    void changeCriteriaActivation(const string &name, bool active);
   
private:

    //This member maps a criterion name with its encoding
    unordered_map<string, string> *mapping;
    
    //This is the double entrance table that contains all the weight.
    // - the index of the list indicate the cardinality of the weight combination.
    // - the Hash contains the pairs criteria_combination<->weight
    // NOTE: all the keys of the String must be sorted by lexicographic order.
    vector<unordered_map<string, double> * > *weights;
    vector<string, bool> *activeCriteria;
    int numOfActiveCriteria;
    int lastInsertedCriteria;
    std::mutex *mutex;
};


#endif // WEIGHTMATRIX_H