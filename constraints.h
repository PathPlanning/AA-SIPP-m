#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <math.h>
#include <unordered_map>
#include "gl_const.h"
#include "structs.h"
#include <algorithm>
#include <iostream>

class Constraints
{
public:
    Constraints(int width, int height, double tweight);
    virtual ~Constraints(){}
    std::vector<std::pair<int,int>> findConflictCells(Node cur);
    void updateSafeIntervals(const std::vector<std::pair<int,int>> &cells, section sec, bool goal);
    std::vector<std::pair<double, double> > getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int w);
    virtual void addConstraints(const std::vector<Node> &sections) = 0;
    virtual std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w) = 0;
    std::pair<double,double> getSafeInterval(int i, int j, int n) {return safe_intervals[i][j][n];}
    virtual void removeStartConstraint(int i, int j) = 0;
    virtual void addStartConstraint(int i, int j, int size) = 0;

protected:
    std::vector<std::vector<std::vector<std::pair<double,double>>>> safe_intervals;
    double tweight;
};

class PointConstraints : public Constraints
{
public:
    PointConstraints(int width, int height, double tweight);
    void addConstraints(const std::vector<Node> &sections);
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w);
private:
    std::vector<std::vector<std::vector<constraint>>> constraints;
    double gap;
    int size;
    void addStartConstraint(int i, int j, int size)
    {
        constraint con;
        con.i = i;
        con.j = j;
        con.goal = false;
        this->size = size;
        for(int k = 0; k < size; k++)
        {
            con.g = k;
            constraints[i][j].insert(constraints[i][j].begin(), con);
        }
        return;
    }
    void removeStartConstraint(int i, int j)
    {
        for(int k = 0; k < size; k++)
            constraints[i][j].erase(constraints[i][j].begin());
        return;
    }
};

class VelocityConstraints : public Constraints
{
public:
    VelocityConstraints(int width, int height, double tweight);
    void addConstraints(const std::vector<Node> &sections);
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w);
private:
    bool hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision);
protected:
    std::vector<std::vector<std::vector<section>>> constraints;
    void addStartConstraint(int i, int j, int size)
    {
        section sec(i, j, i, j, 0, size);
        constraints[i][j].insert(constraints[i][j].begin(),sec);
        return;
    }
    void removeStartConstraint(int i, int j)
    {
        constraints[i][j].erase(constraints[i][j].begin());
        return;
    }
};

class SectionConstraints : public VelocityConstraints
{
public:
    SectionConstraints(int width, int height, double tweight):VelocityConstraints(width, height, tweight){}
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w);
    std::pair<double,double> countInterval(section sec, Node curNode);
    void addStartConstraint(int i, int j, int size)
    {
        section sec(i,j,i,j,0,size);
        constraints[i][j].insert(constraints[i][j].begin(),sec);
        return;
    }
    void removeStartConstraint(int i, int j)
    {
        constraints[i][j].erase(constraints[i][j].begin());
        return;
    }

private:
    int checkIntersection(Point A, Point B, Point C, Point D, Point &intersec);
    double dist(Node A, Node B){return sqrt(pow(A.i - B.i, 2) + pow(A.j - B.j, 2));}
    double dist(Point A, Point B){return sqrt(pow(A.i - B.i, 2) + pow(A.j - B.j, 2));}
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
    double dist(Point A, Point C, Point D)
    {
        return fabs((C.i-D.i)*A.j+(D.j-C.j)*A.i+(C.j*D.i-D.j*C.i))/sqrt(pow(C.i-D.i,2)+pow(C.j-D.j,2));
    }
};

#endif // CONSTRAINTS_H
