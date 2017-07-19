#ifndef STRUCTS_H
#define STRUCTS_H
#include <memory>
#include "gl_const.h"

struct conflict
{
    int agent1;
    int agent2;
    int sec1;
    int sec2;
    double i;
    double j;
    double g;
};

struct constraint
{
    double i;
    double j;
    double g;
    bool goal;
};

struct movement
{
    double g;
    int p_dir;
    int s_dir;
};

struct Node
{
    int     i, j;
    double   F;
    double   g;
    Node*   Parent;
    std::pair<double,double> interval;

    Node()
    {
        i = -1;
        j = -1;
        F = -1;
        g = -1;
        Parent = NULL;
    }

    Node(int i, int j)
    {
        this->i=i;
        this->j=j;
        F=-1;
        g=-1;
        Parent=NULL;
    }

    Node(int x, int y, double f, double G)
    {
        i = x;
        j = y;
        F = f;
        g = G;
        Parent = NULL;
    }

    ~Node()
    {
        Parent = NULL;
    }
};

struct section
{
    int i1;
    int i2;
    int j1;
    int j2;
    double g1;
    double g2;//is needed for goal and wait actions

    bool operator== (const section &comp)   const{return (i1==comp.i1 && j1==comp.j1 && g1==comp.g1);}

    section()
    {
        i1=-1;
        j1=-1;
        i2=-1;
        j2=-1;
        g1=-1;
        g2=-1;
    }

    section(const Node &a, const Node &b)
    {
        i1=a.i;
        j1=a.j;
        g1=a.g;
        i2=b.i;
        j2=b.j;
        g2 = b.g;
    }
};

class Point {
public:
    double i;
    double j;

    Point(double _i = 0.0, double _j =0.0):i (_i), j (_j){}
    //Point(std::pair<double, double> p){i=p.first; j=p.second;}
    Point operator-(Point& p){return Point(i - p.i, j - p.j);}
    int operator== (Point& p){return (i == p.i) && (j == p.j);}
    int classify(Point&pO, Point&p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;//BEYOND;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if (pO == p2)
            return 5;//ORIGIN;
        if (p1 == p2)
            return 6;//DESTINATION;
        return 7;//BETWEEN;
    }
};
#endif
