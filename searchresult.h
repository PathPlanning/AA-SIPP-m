#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H

#include <vector>
#include <list>
#include <structs.h>

struct ResultPathInfo
{
    bool pathfound;
    float pathlength;
    unsigned int nodescreated;
    unsigned int numberofsteps;
    double time;
    std::list<Node> path;
    std::vector<Node> sections;

    ResultPathInfo()
    {
        nodescreated=0;
        numberofsteps=0;
        time=0;
        pathfound = false;
        pathlength = 0;
        path.clear();
        sections.clear();
    }
};

struct SearchResult
{
    bool pathfound;
    double pathlength;
    unsigned int nodescreated;
    unsigned int numberofsteps;
    double time;
    unsigned int agents;
    int agentsSolved;
    std::vector<ResultPathInfo> pathInfo;

    SearchResult() : pathInfo(1)
    {
        pathfound = false;
        pathlength = 0;
        nodescreated = 0;
        numberofsteps = 0;
        time = 0;
        agents = 0;
    }

    ~SearchResult()
    {
        pathInfo.clear();
    }

};

#endif // SEARCHRESULT_H
