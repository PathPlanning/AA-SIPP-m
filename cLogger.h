#ifndef CLOGGER_H
#define CLOGGER_H

#include"cMap.h"
#include <vector>
#include <unordered_map>
#include"searchresult.h"
class cLogger
{	
public:
    float loglevel;

public:
    cLogger();
    virtual ~cLogger();
    virtual bool getLog(const char* FileName) = 0;
    virtual void saveLog() = 0;
    virtual void writeToLogSummary(const SearchResult &sr)=0;
    virtual void writeToLogPath(const SearchResult &sresult)=0;
    virtual void writeToLogMap(const cMap &map, const SearchResult &sresult)=0;
};
#endif
