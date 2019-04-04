#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <vector>
#include "structs.h"
#include "tinyxml2.h"
#include "gl_const.h"
#include "lineofsight.h"

class Map
{
public:
    std::vector<std::vector<int>> Grid;
    unsigned int height, width;

public:
    Map();
    ~Map();
    bool getMap(const char* FileName);
    bool CellIsTraversable (int i, int j) const;
    bool CellOnGrid (int i, int j) const;
    bool CellIsObstacle(int i, int j) const;
    int  getValue(int i, int j) const;
    std::vector<Node> getValidMoves(int i, int j, int k, double size) const;
};

#endif
