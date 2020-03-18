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
    std::vector<Agent> agents;
public:
    Task(){}
    bool getTask(const char* fileName);
    Agent getAgent(unsigned int id) const;
    unsigned int getNumberOfAgents() const;
    void make_ids(int width);
    void make_ij(const Map &map);
    bool validateTask(const Map &map);
};

#endif // TASK_H
