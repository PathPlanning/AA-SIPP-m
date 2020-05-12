#ifndef HEURISTIC_H
#define HEURISTIC_H
#include <vector>
#include <list>
#include <map>
#include "map.h"
#include "structs.h"
#include "lineofsight.h"
class Heuristic
{
    int openSize;
    int agents;
    int width;
    int height;
    std::vector<std::vector<std::vector<double>>> h_values;
    std::vector<std::list<hNode>> open;
    std::list<Node> closed;
    LineOfSight lineofsight;
    std::vector<std::vector<std::pair<int, int>>> parents;
public:
    Heuristic(int _agents=0, int _height=0, int _width=0)
    {
        agents = _agents;
        width = _width;
        height = _height;
        h_values.resize(agents);
        for(int i=0; i<agents; i++)
        {
            h_values[i].resize(height);
            for(int j=0; j<height; j++)
                h_values[i][j].resize(width, -1);
        }
        lineofsight.setSize(0.5);
    }
    double getValue(int task_id, int i, int j) { return h_values[task_id][i][j]; }
    void addOpen(hNode newNode)
    {
        if (open[newNode.i].empty())
        {
            open[newNode.i].push_back(newNode);
            openSize++;
            return;
        }

        std::list<hNode>::iterator iter, pos, delpos;
        bool posFound(false);
        pos = open[newNode.i].end();
        for(iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
        {
            if ((newNode.g - CN_EPSILON < iter->g) && !posFound)
            {
                pos = iter;
                posFound = true;
            }

            if (iter->j == newNode.j)
            {
                if(iter->g - newNode.g < CN_EPSILON)//if existing state dominates new one
                    return;
                if((newNode.g - iter->g) < CN_EPSILON)//if new state dominates the existing one
                {
                    if(pos == iter)
                    {
                        iter->g = newNode.g;
                        iter->p_i = newNode.p_i;
                        iter->p_j = newNode.p_j;
                        return;
                    }
                    delpos = iter;
                    iter--;
                    open[newNode.i].erase(delpos);
                    openSize--;
                }
            }
        }
        open[newNode.i].insert(pos, newNode);
        openSize++;
        return;
    }
    hNode getMin()
    {
        hNode min;
        min.g = std::numeric_limits<double>::max();
        for(int i = 0; i < open.size(); i++)
            if(!open[i].empty() && open[i].begin()->g - CN_EPSILON < min.g)
                min = *open[i].begin();
        return min;
    }
    void count(const Map &map, Task task, int task_id, int connectedness, std::vector<std::pair<int,int>> agents, bool reset)
    {
        openSize = 0;
        open.clear();
        open.resize(height);
        parents.clear();
        parents.resize(height);
        for(int i = 0; i < height; i++)
            parents[i].resize(width, std::make_pair(-1,-1));

        hNode curNode, newNode;
        curNode.i = task.i;
        curNode.j = task.j;
        curNode.g = 0;
        curNode.p_i = -1;
        curNode.p_j = -1;
        addOpen(curNode);
        h_values[task_id][task.i][task.j] = 0;
        while(openSize > 0)
        {
            curNode = getMin();
            open[curNode.i].pop_front();
            openSize--;
            parents[curNode.i][curNode.j] = {curNode.p_i, curNode.p_j};
            //closed.push_back(curNode);
            //auto parent = &closed.back();
            //closed.insert({curNode.i*map.width + curNode.j, curNode});
            h_values[task_id][curNode.i][curNode.j] = curNode.g;
            std::vector<std::pair<int, int>> valid_moves = {{0,1}, {1,0}, {-1,0}, {0,-1}};
            for(auto move: valid_moves)
            {
                if(!map.CellOnGrid(curNode.i + move.first, curNode.j + move.second) || map.CellIsObstacle(curNode.i + move.first, curNode.j + move.second))
                    continue;
                newNode.i = curNode.i + move.first;
                newNode.j = curNode.j + move.second;
                newNode.g = curNode.g + 1.0;
                newNode.p_i = curNode.i;
                newNode.p_j = curNode.j;
                if(reset)
                {
                    newNode = resetParent(newNode, curNode, map, task_id);
                }
                if(h_values[task_id][newNode.i][newNode.j] < 0)
                    addOpen(newNode);
            }
        }
    }
    hNode resetParent(hNode current, hNode Parent, const Map &map, int task_id)
    {
        if(Parent.p_i < 0)
            return current;
        int pp_i = parents[Parent.p_i][Parent.p_j].first;
        int pp_j = parents[Parent.p_i][Parent.p_j].second;
        if(pp_i < 0 || (current.i == pp_i && current.j == pp_j))
            return current;
        if(lineofsight.checkLine(pp_i, pp_j, current.i, current.j, map))
        {
            current.g = h_values[task_id][pp_i][pp_j] + sqrt(pow(pp_i - current.i, 2) + pow(pp_j - current.j,2));
            current.p_i = pp_i;
            current.p_j = pp_j;
        }
        return current;
    }

};

#endif // HEURISTIC_H
