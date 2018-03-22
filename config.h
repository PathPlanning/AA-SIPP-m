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
    float *searchParams;
    int N;
public:
    Config();
    ~Config();

    bool getConfig(const char* FileName);
};

#endif
