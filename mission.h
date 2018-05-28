#ifndef MISSION_H
#define MISSION_H

#include "map.h"
#include "config.h"
#include "search.h"
#include <string>
#include "xmlLogger.h"
#include "searchresult.h"
#include "sipp.h"
#include "aa_sipp.h"

class Mission
{
public:
    Mission(const char* fName);
    ~Mission();

    bool getMap();
    bool getConfig();
    bool createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();

private:
    Map    m_map;
    Config m_config;
    Search *m_pSearch;
    Logger *m_pLogger;
    SearchResult sr;
    const char* m_fileName;

};

#endif

