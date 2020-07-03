#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <unordered_map>
#include "gl_const.h"
#include "structs.h"
#include <algorithm>
#include <iostream>
#include <lineofsight.h>
#include "map.h"
#include "primitive.h"
#include "dynamicobstacles.h"

class Constraints
{
public:
    Constraints(int width, int height);
    ~Constraints(){}
    void updateCellSafeIntervals(std::pair<int, int> cell);
    std::vector<SafeInterval> getSafeIntervals(Node curNode, const ClosedList &close);
    std::vector<SafeInterval> getSafeIntervals(Node curNode);
    void addConstraints(const std::vector<Primitive> &primitives, double size, double mspeed, const Map &map);
    std::vector<SafeInterval> findIntervals(Node curNode, std::vector<double> &EAT, const ClosedList &close, const OpenContainer &open);
    SafeInterval getSafeInterval(int i, int j, int n) {return safe_intervals[i][j][n];}
    void resetSafeIntervals(int width, int height);
    void setSize(double size) {agentsize = size;}
    void setObstacles(DynamicObstacles *obs) {obstacles = obs;}

private:
    void getEAT(Node curNode, double &startTime, double open_node_g);
    std::vector<std::vector<std::vector<int>>> constraints;
    std::vector<std::vector<std::vector<SafeInterval>>> safe_intervals;
    std::vector<std::vector<std::vector<SafeInterval>>> collision_intervals;
    double agentsize;
    int prim_id;
    DynamicObstacles *obstacles;

};


#endif // CONSTRAINTS_H
