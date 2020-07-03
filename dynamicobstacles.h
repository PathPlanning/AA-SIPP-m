#ifndef DYNAMICOBSTACLES_H
#define DYNAMICOBSTACLES_H
#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include "gl_const.h"
#include "structs.h"
#include "tinyxml2.h"

struct obstacle
{
    std::string id;
    double size;
    double mspeed;
    std::vector<Node> sections;
    std::vector<Primitive> primitives;
    obstacle(){ id = -1; size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; }
};

class DynamicObstacles
{
    std::vector<obstacle> obstacles;
    std::vector<Primitive> primitives;
public:
    DynamicObstacles();
    bool getObstacles(const char* fileName);
    std::vector<Node> getSections(int num) const;
    std::vector<Primitive> getPrimitives(int num) const;
    Primitive getPrimitive(int id);
    double getSize(int num) const;
    double getMSpeed(int num) const;
    std::string getID(int num) const;
    int getNumberOfObstacles() const { return obstacles.size(); }
};

#endif // DYNAMICOBSTACLES_H
