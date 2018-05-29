#ifndef AA_SIPP_H
#define AA_SIPP_H

#include "search.h"
#include "constraints.h"
#include <random>
#include <algorithm>
#include "lineofsight.h"

class AA_SIPP : public Search
{

public:

    AA_SIPP(double weight, int rescheduling, int timelimit, int prioritization, int startsafeinterval, int tweight);
    ~AA_SIPP();
    SearchResult startSearch(Map &map);

private:

    void addOpen(Node &newNode);
    Node findMin(int size);
    bool stopCriterion();
    double getCost(int a_i, int a_j, int b_i, int b_j);
    double calcHeading(const Node &node, const Node &son);
    void findSuccessors(const Node curNode, const Map &map, std::list<Node> &succs, int numOfCurAgent);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal);
    void addConstraints(){}
    Node resetParent(Node current, Node Parent, const Map &map);
    bool findPath(int numOfCurAgent, const Map &map);
    std::vector<conflict> CheckConflicts(const Map &map);//bruteforce checker. It splits final(already built) trajectories into sequences of points and checks distances between them
    void setPriorities(const Map& map);
    bool changePriorities(int bad_i);
    double weight;
    bool breakingties;
    int rescheduling;
    int timelimit;
    int prioritization;
    int startsafeinterval;
    double tweight;
    unsigned int closeSize, openSize;
    std::list<Node> lppath;
    std::vector<std::list<Node>> open;
    std::unordered_multimap<int, Node> close;
    std::vector<Node> hppath;
    std::vector<std::vector<int>> priorities;
    std::vector<int> current_priorities;
    LineOfSight lineofsight;
    agent curagent;
    Constraints *constraints;
};

#endif // AA_SIPP_H
