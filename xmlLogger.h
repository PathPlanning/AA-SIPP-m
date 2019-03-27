#ifndef XMLLOGGER_H
#define XMLLOGGER_H

#include <iostream>
#include "tinyxml2.h"
#include <string>
#include "task.h"
#include "config.h"
#include "searchresult.h"
#include <fstream>

class XmlLogger
{
private:
    std::string LogFileName;
    tinyxml2::XMLDocument *doc;
    int loglevel;

public:

    XmlLogger(int loglvl){ loglevel = loglvl; LogFileName = ""; doc = nullptr; }
    ~XmlLogger() { if(doc) { doc->Clear(); delete doc;} }
    bool createLog(const char* FileName);
    void writeToLogInput(const char* taskName, const char* mapName, const char* configName, const char* obstaclesName);
    void writeToLogFile(const char* fileName);
    void writeToLogSummary(const SearchResult &sr);
    void writeToLogPath(const SearchResult &sresult, const Task &task, const Config &config);
    void writeToLogMap(const Map &map, const SearchResult &sresult);
    void saveLog();
};

#endif
