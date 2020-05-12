#ifndef INSTANCE_H
#define INSTANCE_H
#include "structs.h"
#include "tinyxml2.h"
#include <vector>
#include <iostream>
#include <sstream>
#include "map.h"
#include "lineofsight.h"

class Instance
{
    std::vector<Agent> agents;
    std::vector<Task>   tasks;
public:
    Instance(){}
    bool getInstance(const char* fileName, int num);
    Agent getAgent(unsigned int id) const;
    Task getTask(unsigned int id) const;
    unsigned int getNumberOfAgents() const;
    unsigned int getNumberOfTasks() const;
    bool validateTask(const Map &map);
};

#endif // INSTANCE_H
