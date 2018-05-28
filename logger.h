#ifndef LOGGER_H
#define LOGGER_H

#include "map.h"
#include <vector>
#include <unordered_map>
#include "searchresult.h"

class Logger
{
public:
    float loglevel;
    Logger() {loglevel = -1;}
    virtual ~Logger(){}
    virtual bool getLog(const char* FileName) = 0;
    virtual void saveLog() = 0;
    virtual void writeToLogSummary(const SearchResult &sr) = 0;
    virtual void writeToLogPath(const SearchResult &sresult) = 0;
    virtual void writeToLogMap(const Map &map, const SearchResult &sresult) = 0;
};
#endif
