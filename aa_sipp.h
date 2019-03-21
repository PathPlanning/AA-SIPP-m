#ifndef AA_SIPP_H
#define AA_SIPP_H

#include "search.h"
#include "constraints.h"
#include <random>
#include <algorithm>
#include "lineofsight.h"
#include "config.h"
#include <memory>

class AA_SIPP : public Search
{

public:

    AA_SIPP(const Config &config);
    ~AA_SIPP();
    SearchResult startSearch(Map &map, Task &task, DynamicObstacles &obstacles);

private:

    void addOpen(Node &newNode);
    Node findMin(int size);
    bool stopCriterion();
    double getCost(int a_i, int a_j, int b_i, int b_j);
    double calcHeading(const Node &node, const Node &son);
    void findSuccessors(const Node curNode, const Map &map, std::list<Node> &succs);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal);
    void addConstraints(){}
    Node resetParent(Node current, Node Parent, const Map &map);
    bool findPath(unsigned int numOfCurAgent, const Map &map);
    std::vector<conflict> CheckConflicts(const Task &task);//bruteforce checker. It splits final(already built) trajectories into sequences of points and checks distances between them
    void setPriorities(const Task &task);
    bool changePriorities(int bad_i);
    unsigned int closeSize, openSize;
    std::list<Node> lppath;
    std::vector<std::list<Node>> open;
    std::unordered_multimap<int, Node> close;
    std::vector<Node> hppath;
    std::vector<std::vector<int>> priorities;
    std::vector<int> current_priorities;
    LineOfSight lineofsight;
    Agent curagent;
    Constraints *constraints;
    std::shared_ptr<const Config> config;
};

#endif // AA_SIPP_H
