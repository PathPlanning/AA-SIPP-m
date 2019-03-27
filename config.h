#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include "tinyxml2.h"
#include <string>
#include "gl_const.h"
#include <algorithm>
#include <sstream>


class Config
{
public:
    Config(){ loglevel = CN_LOGLVL_NORM; }
    int loglevel;
    int metrictype;
    bool allowanyangle;
    bool planforturns;
    double hweight;
    double timelimit;
    int rescheduling;
    int initialprioritization;
    double startsafeinterval;
    double additionalwait;
    std::string logfilename;
    std::string logpath;

    bool getConfig(const char* fileName);
};

#endif
