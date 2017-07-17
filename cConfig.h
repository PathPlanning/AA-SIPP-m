#ifndef CCONFIG_H
#define CCONFIG_H

#include <iostream>
#include "tinyxml.h"
#include "tinystr.h"
#include <string>
#include "gl_const.h"
#include <algorithm>
#include <sstream>


class cConfig
{
public:
    float *searchParams;
    int N;
public:
    cConfig();
    ~cConfig();

    bool getConfig(const char* FileName);
};

#endif
