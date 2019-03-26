#ifndef SEARCH_H
#define SEARCH_H

#include "map.h"
#include "xmlLogger.h"
#include "gl_const.h"
#include "searchresult.h"
#include <math.h>
#include "task.h"
#include "dynamicobstacles.h"
#include <unordered_map>
#ifdef __linux__
    #include <sys/time.h>
#else
    #include <windows.h>
#endif

class Search
{
public:
    Search(){}
    virtual ~Search(){}
    virtual void addOpen(Node& newNode) = 0;
    virtual SearchResult startSearch(Map &map, Task &task, DynamicObstacles &obstacles) = 0;
    virtual bool findPath(unsigned int numOfCurAgent, const Map &map) = 0;
    virtual std::list<Node> findSuccessors(const Node curNode, const Map &map) = 0;
    virtual Node findMin(int size) = 0;
    virtual bool stopCriterion(const Node &curNode, Node &goalNode) = 0;
    virtual void makePrimaryPath(Node curNode) = 0;
    virtual void makeSecondaryPath(Node curNode) = 0;
    virtual void addConstraints() = 0;
    virtual std::vector<conflict> CheckConflicts(const Task &task) = 0;
    SearchResult sresult;
};

#endif
