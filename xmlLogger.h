#ifndef XMLLOGGER_H
#define XMLLOGGER_H

#include "logger.h"
#include <iostream>
#include "tinyxml2.h"
#include <string>
#include "searchresult.h"
#include <fstream>
class XmlLogger:public Logger
{

private:

    std::string LogFileName;
    tinyxml2::XMLDocument *doc;

public:

    XmlLogger(float loglvl);
    ~XmlLogger();
    bool createLog(const char* FileName);
    void writeToLogInput(const char* taskName, const char* mapName, const char* configName, const char* obstaclesName);
    void writeToLogSummary(const SearchResult &sr);
    void writeToLogPath(const SearchResult &sresult, const Task &task, const Config &config);
    void writeToLogMap(const Map &map, const SearchResult &sresult);
    void saveLog();
};

#endif
