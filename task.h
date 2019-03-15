#ifndef TASK_H
#define TASK_H
#include "structs.h"
#include "tinyxml2.h"
#include <vector>
#include <iostream>
#include <sstream>
#include "map.h"
#include "lineofsight.h"

class Task
{
    std::vector<agent> agents;
public:
    Task(){}
    bool getTask(const char* fileName);
    agent getAgent(unsigned int id) const;
    unsigned int getNumberOfAgents() const;
    bool validateTask(const Map &map);
};

#endif // TASK_H
