#ifndef XMLLOGGER_H
#define XMLLOGGER_H

#include "logger.h"
#include <iostream>
#include "tinyxml2.h"
#include <string>
#include "searchresult.h"

class XmlLogger:public Logger
{

private:

    std::string LogFileName;
    tinyxml2::XMLDocument *doc;

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
