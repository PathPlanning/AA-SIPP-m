#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <math.h>
#include <unordered_map>
#include "gl_const.h"
#include "structs.h"
#include <algorithm>
#include <iostream>
#include <lineofsight.h>

class Constraints
{
public:
    Constraints(int width, int height, double tweight);
    virtual ~Constraints(){}
    virtual void updateCellSafeIntervals(std::pair<int, int> cell) = 0;
    std::vector<std::pair<double, double> > getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int w);
    std::vector<std::pair<double, double> > getSafeIntervals(Node curNode);
    virtual void addConstraints(const std::vector<Node> &sections, double size) = 0;
    virtual std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w) = 0;
    std::pair<double,double> getSafeInterval(int i, int j, int n) {return safe_intervals[i][j][n];}
    void resetSafeIntervals(int width, int height);
    virtual void removeStartConstraint(int i, int j, std::vector<std::pair<int, int>> cells) = 0;
    virtual void addStartConstraint(int i, int j, int size, std::vector<std::pair<int, int>> cells, double agentsize = 0.5) = 0;
    void setSize(double size) {agentsize = size;}
    double minDist(Point A, Point C, Point D)
    {
        int classA=A.classify(C,D);
        if(classA==3)
            return sqrt(pow(A.i-C.i,2)+pow(A.j-C.j,2));
        else if(classA==4)
            return sqrt(pow(A.i-D.i,2)+pow(A.j-D.j,2));
        else
            return fabs((C.i-D.i)*A.j+(D.j-C.j)*A.i+(C.j*D.i-D.j*C.i))/sqrt(pow(C.i-D.i,2)+pow(C.j-D.j,2));
    }

protected:
    std::vector<std::vector<std::vector<std::pair<double,double>>>> safe_intervals;
    double tweight;
    double agentsize;

};


class VelocityConstraints : public Constraints
{
public:
    VelocityConstraints(int width, int height, double tweight);
    void addConstraints(const std::vector<Node> &sections, double size);
    void updateCellSafeIntervals(std::pair<int, int> cell);
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w);
private:
    bool hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision);
protected:
    std::vector<std::vector<std::vector<section>>> constraints;
    void addStartConstraint(int i, int j, int size, std::vector<std::pair<int, int>> cells, double agentsize = 0.5)
    {
        section sec(i, j, i, j, 0, size);
        sec.size = agentsize;
        for(auto cell: cells)
            constraints[cell.first][cell.second].insert(constraints[cell.first][cell.second].begin(),sec);
        return;
    }
    void removeStartConstraint(int i, int j, std::vector<std::pair<int, int>> cells)
    {
        for(auto cell: cells)
            constraints[cell.first][cell.second].erase(constraints[cell.first][cell.second].begin());
        return;
    }
};


#endif // CONSTRAINTS_H
