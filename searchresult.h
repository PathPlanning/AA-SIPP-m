#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H

#include <vector>
#include <list>
#include <structs.h>

struct ResultPathInfo
{
    bool pathfound;
    double pathlength;
    double runtime;
    std::list<Node> path;
    std::vector<Node> sections;

    ResultPathInfo()
    {
        runtime = 0;
        pathfound = false;
        pathlength = 0;
        path.clear();
        sections.clear();
    }
};

struct SearchResult
{
    bool pathfound;
    double makespan;
    double flowtime;
    double runtime;
    unsigned int agents;
    int agentsSolved;
    int tries;
    std::vector<ResultPathInfo> pathInfo;

    SearchResult() : pathInfo(1)
    {
        pathfound = false;
        runtime = 0;
        flowtime = 0;
        makespan = 0;
        agents = 0;
    }

    ~SearchResult()
    {
        pathInfo.clear();
    }

};

#endif // SEARCHRESULT_H
