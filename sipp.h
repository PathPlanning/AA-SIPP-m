#ifndef SIPP_H
#define SIPP_H

#include "cSearch.h"

class SIPP : public cSearch
{

public:

    SIPP(double weight, int metrictype, bool breakingties);
    ~SIPP();
    SearchResult startSearch(cLogger *Log, cMap &Map);

private:

    bool findPath(int numOfCurAgent, const cMap &Map);
    void findSuccessors(const Node curNode, const cMap &Map, std::list<Node> &succs, int numOfCurAgent);
    void addOpen(Node &newNode);
    Node findMin(int size);
    double countHValue(int i, int j, int goal_i, int goal_j);
    bool stopCriterion();
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    void addConstraints();
    std::vector<conflict> CheckConflicts();

    double weight;
    int metrictype;
    bool breakingties;
    unsigned int closeSize, openSize;
    std::unordered_multimap<int, Node> close;
    std::list<Node> *open, lppath;
    std::vector<Node> hppath;
    std::vector<std::vector<std::vector<movement>>> ctable;
};

#endif // SIPP_H
