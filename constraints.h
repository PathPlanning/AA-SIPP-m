#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <unordered_map>
#include "gl_const.h"
#include "structs.h"
#include <algorithm>
#include <iostream>
#include <lineofsight.h>

class Constraints
{
public:
    Constraints(int width, int height);
    virtual ~Constraints(){}
    virtual void updateCellSafeIntervals(std::pair<int, int> cell) = 0;
    std::vector<std::pair<double, double> > getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int w);
    std::vector<std::pair<double, double> > getSafeIntervals(Node curNode);
    virtual void addConstraints(const std::vector<Node> &sections, double size) = 0;
    virtual std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w) = 0;
    std::pair<double,double> getSafeInterval(int i, int j, int n) {return safe_intervals[i][j][n];}
    void resetSafeIntervals(int width, int height);
    virtual void addStartConstraint(int i, int j, int size, std::vector<std::pair<int, int>> cells, double agentsize = 0.5) = 0;
    virtual void removeStartConstraint(std::vector<std::pair<int, int>> cells) = 0;
    void setSize(double size) {agentsize = size;}
    void setParams(double size, double mspeed, double rspeed, double tweight) { agentsize = size; this->mspeed = mspeed; this->rspeed = rspeed; this->tweight = tweight; }
    double minDist(Point A, Point C, Point D);


protected:
    std::vector<std::vector<std::vector<std::pair<double,double>>>> safe_intervals;
    double rspeed;
    double mspeed;
    double agentsize;
    double tweight;

};


class VelocityConstraints : public Constraints
{
public:
    VelocityConstraints(int width, int height);
    void addConstraints(const std::vector<Node> &sections, double size);
    void updateCellSafeIntervals(std::pair<int, int> cell);
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w);
    void addStartConstraint(int i, int j, int size, std::vector<std::pair<int, int>> cells, double agentsize = 0.5);
    void removeStartConstraint(std::vector<std::pair<int, int>> cells);
private:
    bool hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision);
    std::vector<std::vector<std::vector<section>>> constraints;
};


#endif // CONSTRAINTS_H
