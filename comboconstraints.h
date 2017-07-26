#ifndef COMBOCONSTRAINTS_H
#define COMBOCONSTRAINTS_H
#include <vector>
#include <math.h>
#include <unordered_map>
#include "gl_const.h"
#include "Vector2D.h"
#include "structs.h"
#include <algorithm>
#include <iostream>
#include "constraints.h"

class ComboConstraints : public Constraints
{
public:
    ComboConstraints(int width, int height);
    void addConstraints(const std::vector<Node> &sections);
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w);
private:
    std::pair<double,double> countInterval(section sec, Node curNode);
    bool hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision);
    //std::vector<std::vector<std::vector<std::pair<double,double>>>> safe_intervals;
    std::vector<std::vector<std::vector<constraint>>> p_constraints;
    std::vector<std::vector<std::vector<section>>> s_constraints;
    double gap;
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
};
#endif // COMBOCONSTRAINTS_H
