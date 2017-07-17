#ifndef CSEARCH_H
#define CSEARCH_H

#include "cMap.h"
#include "cLogger.h"
#include "cXmlLogger.h"
#include "gl_const.h"
#include "searchresult.h"
#include <math.h>
#include "Vector2D.h"
#ifdef __linux__
    #include <sys/time.h>
#else
    #include <windows.h>
#endif
#include <unordered_map>

class cSearch
{
public:
    cSearch(){};
    virtual ~cSearch(){};
    virtual void addOpen(Node& newNode) = 0;
    virtual SearchResult startSearch(cLogger *Log, cMap &Map) = 0;
    virtual bool findPath(int numOfCurAgent, const cMap &Map) = 0;
    virtual void findSuccessors(const Node curNode, const cMap &Map, std::list<Node> &succs, int numOfCurAgent) = 0;
    virtual Node findMin(int size) = 0;
    virtual bool stopCriterion() = 0;
    virtual void makePrimaryPath(Node curNode) = 0;
    virtual void makeSecondaryPath(Node curNode) = 0;
    virtual void addConstraints() = 0;
    virtual std::vector<conflict> CheckConflicts() = 0;
    SearchResult sresult;
};

#endif
