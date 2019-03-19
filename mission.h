#ifndef MISSION_H
#define MISSION_H

#include "map.h"
#include "config.h"
#include "search.h"
#include "xmlLogger.h"
#include "searchresult.h"
#include "aa_sipp.h"
#include "task.h"

class Mission
{
public:
    Mission();
    ~Mission();

    bool getMap();
    bool getTask();
    bool getConfig();
    bool createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();
    void setFileNames(const char* mapName, const char *taskName, const char *configName);

private:
    Map     m_map;
    Task    m_task;
    Config  m_config;
    Search  *m_pSearch;
    Logger  *m_pLogger;
    SearchResult sr;
    const char* mapName;
    const char* taskName;
    const char* configName;

};

#endif

