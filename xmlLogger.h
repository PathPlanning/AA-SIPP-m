#ifndef XMLLOGGER_H
#define XMLLOGGER_H

#include "logger.h"
#include <iostream>
#include "tinyxml.h"
#include "tinystr.h"
#include <string>
#include "searchresult.h"
#include <sstream>

class XmlLogger:public Logger
{

private:

    std::string LogFileName;
    TiXmlDocument *doc;

public:

    XmlLogger(float loglvl);
    ~XmlLogger();
    bool getLog(const char* FileName);
    void saveLog();
    void writeToLogSummary(const SearchResult &sr);
    void writeToLogPath(const SearchResult &sresult);
    void writeToLogMap(const Map &map, const SearchResult &sresult);
};

#endif
