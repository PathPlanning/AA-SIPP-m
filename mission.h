#ifndef MISSION_H
#define MISSION_H

#include "map.h"
#include "config.h"
#include "xmlLogger.h"
#include "searchresult.h"
#include "aa_sipp.h"
#include "instance.h"
#include "dynamicobstacles.h"

class Mission
{
public:
    Mission();
    ~Mission();

    bool getMap();
    bool getTask(int a);
    bool getConfig();
    bool getObstacles();
    void createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();
    void setFileNames(const char *taskName, const char* mapName, const char *configName, const char *obstaclesName);

private:
    Map              m_map;
    Instance         m_task;
    Config           m_config;
    DynamicObstacles m_obstacles;
    AA_SIPP*         m_pSearch;
    XmlLogger*       m_pLogger;
    SearchResult     sr;
    const char*      mapName;
    const char*      taskName;
    const char*      configName;
    const char*      obstaclesName;

};

#endif

