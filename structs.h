#ifndef STRUCTS_H
#define STRUCTS_H
#include <memory>

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
    int agent;
    bool goal;
    double direction;
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
    float   F;
    float   g;
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

    Node(int x, int y, float f, float G)
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

#endif
