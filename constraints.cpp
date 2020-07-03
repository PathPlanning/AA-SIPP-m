#include "constraints.h"

Constraints::Constraints(int width, int height)
{
    safe_intervals.resize(height);
    collision_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        collision_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            collision_intervals[i][j].clear();
            safe_intervals[i][j].resize(0);
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
    constraints.resize(height);
    for(int i = 0; i < height; i++)
    {
        constraints[i].resize(width);
        for(int j = 0; j < width; j++)
            constraints[i][j].resize(0);
    }
    prim_id=0;
}

void Constraints::resetSafeIntervals(int width, int height)
{
    safe_intervals.resize(height);
    collision_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        collision_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            collision_intervals[i][j].clear();
            safe_intervals[i][j].resize(0);
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
}

void Constraints::updateCellSafeIntervals(std::pair<int, int> cell)
{
    LineOfSight los(agentsize);
    std::vector<std::pair<int, int>> cells = los.getCells(cell.first, cell.second);
    std::vector<int> prim_ids;
    for(int k = 0; k < cells.size(); k++)
        for(int l = 0; l < constraints[cells[k].first][cells[k].second].size(); l++)
            if(std::find(prim_ids.begin(), prim_ids.end(), constraints[cells[k].first][cells[k].second][l]) == prim_ids.end())
                prim_ids.push_back(constraints[cells[k].first][cells[k].second][l]);
    std::vector<Primitive> prims;
    for(int i:prim_ids)
        prims.push_back(obstacles->getPrimitive(i));
    for(int k = 0; k < prims.size(); k++)
    {
        Primitive prim = prims[k];
        double radius = agentsize + prim.size();
        std::pair<double, double> p = prim.getInterval(cell.first, cell.second, radius);
        if(p.first < prim.begin)
            continue;
        SafeInterval interval(p.first, p.second);
        int i2(cell.first), j2(cell.second);
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
        {
            if(safe_intervals[i2][j2][j].begin < interval.begin + CN_EPSILON && safe_intervals[i2][j2][j].end + CN_EPSILON > interval.begin)
            {
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++;
                    if(safe_intervals[i2][j2][j].end < interval.end)
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    else
                        safe_intervals[i2][j2][j].begin = interval.end;
                }
                else if(safe_intervals[i2][j2][j].end < interval.end)
                    safe_intervals[i2][j2][j].end = interval.begin;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][j].begin;
                    new1.second = interval.begin;
                    new2.first = interval.end;
                    new2.second = safe_intervals[i2][j2][j].end;
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    if(new2.first < CN_INFINITY)
                        safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new2.first, new2.second));
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new1.first, new1.second));
                }
            }
            else if(safe_intervals[i2][j2][j].begin > interval.begin - CN_EPSILON && safe_intervals[i2][j2][j].begin < interval.end)
            {
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++;
                }
                if(safe_intervals[i2][j2][j].end < interval.end)
                {
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                }
                else
                {
                    safe_intervals[i2][j2][j].begin = interval.end;
                }
            }
        }
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
            safe_intervals[i2][j2][j].id = j;

        if(safe_intervals[i2][j2][0].begin > 0)
            collision_intervals[i2][j2].push_back({0, safe_intervals[i2][j2][0].begin});
        for(int i=0; i<safe_intervals[i2][j2].size()-1; i++)
            collision_intervals[i2][j2].push_back({safe_intervals[i2][j2][i].end, safe_intervals[i2][j2][i+1].begin});
        if(safe_intervals[i2][j2].back().end < CN_INFINITY)
            collision_intervals[i2][j2].push_back({safe_intervals[i2][j2].back().end, CN_INFINITY});

    }
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode, const ClosedList &close)
{
    std::vector<SafeInterval> intervals(0);
    for(unsigned int i = 0; i < safe_intervals[curNode.i][curNode.j].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][i].end >= curNode.g
                && safe_intervals[curNode.i][curNode.j][i].begin <= (curNode.Parent->interval.end + curNode.g - curNode.Parent->g))
        {
            auto has = close.get<0>().find(boost::make_tuple(curNode.i, curNode.j, safe_intervals[curNode.i][curNode.j][i].id, curNode.angle_id, curNode.speed));
            if(has == close.get<0>().end())
                intervals.push_back(safe_intervals[curNode.i][curNode.j][i]);
        }
    return intervals;
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode)
{
    return safe_intervals[curNode.i][curNode.j];
}

void Constraints::addConstraints(const std::vector<Primitive> &primitives, double size, double mspeed, const Map &map)
{
    if(primitives.size() == 1)
        safe_intervals[primitives.back().source.i][primitives.back().source.j].clear();
    for(auto prim: primitives)
        for(auto c: prim.getCells())
            constraints[c.i][c.j].push_back(prim.id);
}

std::vector<SafeInterval> Constraints::findIntervals(Node curNode, std::vector<double> &EAT, const ClosedList &close, const OpenContainer &open)
{
    std::vector<SafeInterval> curNodeIntervals = getSafeIntervals(curNode, close);
    std::vector<SafeInterval> result;
    if(curNodeIntervals.empty())
    {
        return curNodeIntervals;
    }
    EAT.clear();
    for(unsigned int i=0; i<curNodeIntervals.size(); i++)
    {
        SafeInterval cur_interval(curNodeIntervals[i]);
        curNode.interval = cur_interval;
        if(cur_interval.begin < curNode.g)
            cur_interval.begin = curNode.g;
        double startTime = curNode.Parent->g;
        if(cur_interval.begin > curNode.g)
        {
            if(curNode.Parent->speed > 0)
                continue;
            startTime = cur_interval.begin - curNode.primitive.duration;
        }
        curNode.interval = cur_interval;
        Node open_node = open.findNode(curNode);
        if(open_node.g - CN_EPSILON < startTime + curNode.primitive.duration)
            continue;
        double initTime(startTime);
        getEAT(curNode, startTime, open_node.g);
        if(startTime > curNode.Parent->interval.end || startTime + curNode.primitive.duration > cur_interval.end || (curNode.Parent->speed > 0 && startTime > initTime + CN_EPSILON) || startTime + curNode.primitive.duration > open_node.g)
            continue;
        EAT.push_back(startTime + curNode.primitive.duration);
        result.push_back(curNodeIntervals[i]);
    }
    return result;
}

void Constraints::getEAT(Node curNode, double& startTime, double open_node_g)
{
    auto cells = curNode.primitive.getCells();
    for(int k = 0; k < cells.size(); k++)
    {
        auto c = cells[k];
        std::pair<double, double> interval = {startTime + c.interval.first, startTime + c.interval.second};
        std::vector<SafeInterval> intervals = collision_intervals[c.i][c.j];
        for(int i = 0; i < intervals.size(); i++)
            if((interval.first <= intervals[i].begin && interval.second > intervals[i].begin) ||
                    (interval.first >= intervals[i].begin && interval.first < intervals[i].end))
            {
                startTime = startTime - interval.first + intervals[i].end + CN_EPSILON;
                if(startTime > curNode.Parent->interval.end || startTime + curNode.primitive.duration > curNode.interval.end || curNode.Parent->speed > 0 || startTime + curNode.primitive.duration > open_node_g)
                    return;
                k=-1;
                break;
            }
    }
    return;
}
