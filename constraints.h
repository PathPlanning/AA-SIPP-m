#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <math.h>
#include <unordered_map>
#include "gl_const.h"
#include "Vector2D.h"
#include "structs.h"
#include <algorithm>
#include <iostream>

class Constraints
{
public:
    Constraints(int width, int height);
    virtual ~Constraints(){}
    std::vector<std::pair<int,int>> findConflictCells(Node cur);
    void updateSafeIntervals(const std::vector<std::pair<int,int>> &cells, section sec, bool goal);
    std::vector<std::pair<double, double> > getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int w);
    virtual void addConstraints(const std::vector<Node> &sections) = 0;
    virtual std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w) = 0;
protected:
    std::vector<std::vector<std::vector<std::pair<double,double>>>> safe_intervals;

};

class PointConstraints : public Constraints
{
public:
    PointConstraints(int width, int height);
    void addConstraints(const std::vector<Node> &sections);
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w);
private:
    //std::vector<std::vector<std::vector<std::pair<double,double>>>> safe_intervals;
    std::vector<std::vector<std::vector<constraint>>> constraints;
    double gap;
};

class VelocityConstraints : public Constraints
{
public:
    VelocityConstraints(int width, int height);
    void addConstraints(const std::vector<Node> &sections);
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w);
private:
    bool hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision);
    std::vector<std::vector<std::vector<section>>> constraints;
};
#endif // CONSTRAINTS_H
