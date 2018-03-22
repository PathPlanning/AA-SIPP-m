#ifndef SEARCH_H
#define SEARCH_H

#include "map.h"
#include "logger.h"
#include "xmlLogger.h"
#include "gl_const.h"
#include "searchresult.h"
#include <math.h>
#ifdef __linux__
    #include <sys/time.h>
#else
    #include <windows.h>
#endif
#include <unordered_map>

class Search
{
public:
    Search(){}
    virtual ~Search(){}
    virtual void addOpen(Node& newNode) = 0;
    virtual SearchResult startSearch(Map &map) = 0;
    virtual bool findPath(int numOfCurAgent, const Map &map) = 0;
    virtual void findSuccessors(const Node curNode, const Map &map, std::list<Node> &succs, int numOfCurAgent) = 0;
    virtual Node findMin(int size) = 0;
    virtual bool stopCriterion() = 0;
    virtual void makePrimaryPath(Node curNode) = 0;
    virtual void makeSecondaryPath(Node curNode) = 0;
    virtual void addConstraints() = 0;
    virtual std::vector<conflict> CheckConflicts() = 0;
    SearchResult sresult;
};

#endif
