#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <fstream>
#include "tinyxml2.h"
#include "gl_const.h"

class Map
{
public:
    int **Grid;
    int height, width;
    int *start_i, *start_j;
    int *goal_i, *goal_j;
    int agents;

public:
    Map();
    ~Map();
    bool getMap(const char* FileName);
    bool CellIsTraversable (int i, int j) const;
    bool CellOnGrid (int i, int j) const;
    bool CellIsObstacle(int i, int j) const;
    int  getValue(int i, int j) const;
    void addConstraint(int i,int j);
    void removeConstraint(int i,int j);

};

#endif
