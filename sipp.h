#ifndef SIPP_H
#define SIPP_H

#include "search.h"

class SIPP : public Search
{

public:

    SIPP(double weight, int metrictype);
    ~SIPP();
    SearchResult startSearch(Map &map, Task &task);

private:

    bool findPath(unsigned int numOfCurAgent, const Map &map);
    void findSuccessors(const Node curNode, const Map &map, std::list<Node> &succs);
    void addOpen(Node &newNode);
    Node findMin(int size);
    double countHValue(int i, int j, int goal_i, int goal_j);
    bool stopCriterion();
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    void addConstraints();
    std::vector<conflict> CheckConflicts(const Task &task);

    double weight;
    int metrictype;
    unsigned int closeSize, openSize;
    std::unordered_multimap<int, Node> close;
    std::list<Node> *open, lppath;
    std::vector<Node> hppath;
    std::vector<std::vector<std::vector<movement>>> ctable;
    agent curagent;
};

#endif // SIPP_H
