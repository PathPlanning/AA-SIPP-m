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
    Map();
    ~Map();
    bool get_map(const char* FileName);
    bool get_grid(const char* FileName);
    bool get_roadmap(const char* FileName);
    bool CellIsTraversable (int i, int j) const;
    bool CellOnGrid (int i, int j) const;
    bool CellIsObstacle(int i, int j) const;
    int  getValue(int i, int j) const;
    std::vector<Node> getValidMoves(Node curNode, int k, double size) const;
    gNode get_gNode(int id) const {if(id < nodes.size()) return nodes[id]; return gNode();}
    bool is_roadmap() const {return map_is_roadmap;}
    std::pair<int, int> get_ij(int id) const {return {id/width, id%width};}
    int get_id(int i, int j) const {return i*width+j;}

    std::vector<std::vector<int>> Grid;
    std::vector<gNode> nodes;
    std::vector<std::vector<Node>> valid_moves;
    unsigned int height, width, size;
    bool map_is_roadmap;
};

#endif
