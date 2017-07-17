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
#endif
