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
    Config(){ loglevel = CN_LOGLVL_HIGH; }
    int loglevel;
    int metrictype;
    bool allowanyangle;
    double hweight;
    double tweight;
    double timelimit;
    int rescheduling;
    int initialprioritization;
    int startsafeinterval;

    bool getConfig(const char* fileName);
};

#endif
