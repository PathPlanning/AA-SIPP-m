#ifndef CMISSION_H
#define CMISSION_H

#include "cMap.h"
#include "cConfig.h"
#include "cSearch.h"
#include <string>
#include "cXmlLogger.h"
#include "searchresult.h"
#include "sipp.h"
#include "aa_sipp.h"

class cMission
{
public:
    cMission(const char* fName);
    ~cMission();

    bool getMap();
    bool getConfig();
    bool createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();

private:
    cMap    m_map;
    cConfig m_config;

    cSearch *m_pSearch;
    cLogger *m_pLogger;

    const char* m_fileName;

    SearchResult sr;
};

#endif

