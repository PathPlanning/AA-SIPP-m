#ifndef CXMLLOGGER_H
#define CXMLLOGGER_H

#include "cLogger.h"
#include <iostream>
#include "tinyxml.h"
#include "tinystr.h"
#include <string>
#include "searchresult.h"
#include <sstream>

class cXmlLogger:public cLogger
{

private:

    std::string LogFileName;
    TiXmlDocument *doc;

public:

    cXmlLogger(float loglvl);
    ~cXmlLogger();
    bool getLog(const char* FileName);
    void saveLog();
    void writeToLogSummary(const SearchResult &sr);
    void writeToLogPath(const SearchResult &sresult);
    void writeToLogMap(const cMap &map, const SearchResult &sresult);
};

#endif
