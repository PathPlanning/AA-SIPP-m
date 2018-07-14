#ifndef SIPP_H
#define SIPP_H

#include "search.h"

class SIPP : public Search
{

public:

    SIPP(double weight, int metrictype);
    ~SIPP();
    SearchResult startSearch(Map &map);

private:

    bool findPath(int numOfCurAgent, const Map &map);
    void findSuccessors(const Node curNode, const Map &map, std::list<Node> &succs, int numOfCurAgent);
    void addOpen(Node &newNode);
    Node findMin(int size);
    double countHValue(int i, int j, int goal_i, int goal_j);
    bool stopCriterion();
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    void addConstraints(){}
    void addConstraints(std::vector<Node> sections);
    std::vector<conflict> CheckConflicts();
    void repairPaths(const Map &map);

    double weight;
    int metrictype;
    unsigned int closeSize, openSize;
    std::unordered_multimap<int, Node> close;
    std::list<Node> lppath;
    std::vector<std::list<Node>> open;
    std::vector<Node> hppath;
    std::vector<std::vector<std::vector<movement>>> ctable;
    std::vector<int> current_priorities;
};

#endif // SIPP_H
