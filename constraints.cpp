#include "constraints.h"

Constraints::Constraints(int width, int height)
{
    safe_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            safe_intervals[i][j].resize(0);
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
    this->agentsize = sqrt(2.0)/4.0;
}

bool sort_function(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}

void Constraints::resetSafeIntervals(int width, int height)
{
    safe_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            safe_intervals[i][j].resize(0);
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
}

void VelocityConstraints::updateCellSafeIntervals(std::pair<int, int> cell)
{
    if(safe_intervals[cell.first][cell.second].size() > 1)
        return;
    LineOfSight los(agentsize);
    std::vector<std::pair<int, int>> cells = los.getCells(cell.first,cell.second);
    std::vector<section> secs;
    for(int k = 0; k < cells.size(); k++)
        for(int l = 0; l < constraints[cells[k].first][cells[k].second].size(); l++)
        {
            section tempsec = constraints[cells[k].first][cells[k].second][l];
            if(std::find(secs.begin(), secs.end(), tempsec) == secs.end())
                secs.push_back(tempsec);
        }
    for(int k = 0; k < secs.size(); k++)
    {
        section sec = secs[k];
        double radius = agentsize + sec.size;
        int i0(secs[k].i1), j0(secs[k].j1), i1(secs[k].i2), j1(secs[k].j2), i2(cell.first), j2(cell.second);
        std::pair<double,double> interval;
        double dist;
        if(i0 == i1 && j0 == j1)
            dist = 0;
        else
            dist = minDist(Point(i2,j2), Point(i0,j0), Point(i1,j1));
        if(dist >= radius)
            continue;
        int da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
        int db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);

        double ha = sqrt(da - dist*dist);
        double size = sqrt(radius*radius - dist*dist);
        if(da < radius*radius)
        {
            if(db < radius*radius)
            {
                interval.first = sec.g1;
                interval.second = sec.g2;
            }
            else
            {
                double hb = sqrt(db - dist*dist);
                interval.first = sec.g1;
                interval.second = sec.g2 - hb + size;
            }
        }
        else
        {
            if(db < radius*radius)
            {
                interval.first = sec.g1 + ha - size;
                interval.second = sec.g2;
            }
            else
            {
                interval.first = sec.g1 + ha - size;
                interval.second = sec.g1 + ha + size;
            }
        }
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
        {
            if(safe_intervals[i2][j2][j].first <= interval.first && safe_intervals[i2][j2][j].second >= interval.first)
            {
                if(safe_intervals[i2][j2][j].first == interval.first)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, {safe_intervals[i2][j2][j].first,safe_intervals[i2][j2][j].first});
                    j++;
                    if(safe_intervals[i2][j2][j].second < interval.second)
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    else
                        safe_intervals[i2][j2][j].first = interval.second;
                }
                else if(safe_intervals[i2][j2][j].second < interval.second)
                    safe_intervals[i2][j2][j].second = interval.first;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][j].first;
                    new1.second = interval.first;
                    new2.first = interval.second;
                    new2.second = safe_intervals[i2][j2][j].second;
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, new2);
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, new1);
                }
            }
            else if(safe_intervals[i2][j2][j].first >= interval.first && safe_intervals[i2][j2][j].first < interval.second)
            {
                if(safe_intervals[i2][j2][j].first == interval.first)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, {safe_intervals[i2][j2][j].first,safe_intervals[i2][j2][j].first});
                    j++;
                }
                if(safe_intervals[i2][j2][j].second < interval.second)
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                else
                    safe_intervals[i2][j2][j].first = interval.second;
            }
        }
    }
}

void Constraints::updateSafeIntervals(const std::vector<std::pair<int, int> > &cells, section sec, bool goal)
{
    int i0(sec.i1), j0(sec.j1), i1(sec.i2), j1(sec.j2), i2, j2;
    for(unsigned int i = 0; i < cells.size(); i++)
    {
        i2 = cells[i].first;
        j2 = cells[i].second;
        std::pair<double,double> ps, pg, interval;
        ps = {i0, j0};
        pg = {i1, j1};
        double dist = fabs((i0 - i1)*j2 + (j1 - j0)*i2 + (j0*i1 - i0*j1))/sqrt(pow(i0 - i1, 2) + pow(j0 - j1, 2));
        if(dist>=sqrt(2.0)/2.0)
            continue;
        int da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
        int db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
        double ha = sqrt(da - dist*dist);
        double size = sqrt(sqrt(2.0)/2 - dist*dist);
        if(da == 0 && db == 0)
        {
            interval.first = sec.g1;
            interval.second = sec.g2;
        }
        else if(da != 0.0 && db != 0.0)
        {
            interval.first = sec.g1 + ha - size;
            interval.second = sec.g1 + ha + size;
        }
        else if(da == 0.0)
        {
            double hb = sqrt(db - dist*dist);
            interval.first = sec.g1;
            interval.second = sec.g2 - hb + size;
        }
        else
        {
            interval.first = sec.g1 + ha - size;
            interval.second = sec.g2;
            if(goal)
                interval.second = CN_INFINITY;
        }
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
        {
            if(safe_intervals[i2][j2][j].first <= interval.first && safe_intervals[i2][j2][j].second >= interval.first)
            {
                if(safe_intervals[i2][j2][j].first == interval.first)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, {safe_intervals[i2][j2][j].first,safe_intervals[i2][j2][j].first});
                    j++;
                    if(safe_intervals[i2][j2][j].second < interval.second)
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    else
                        safe_intervals[i2][j2][j].first = interval.second;
                }
                else if(safe_intervals[i2][j2][j].second < interval.second)
                    safe_intervals[i2][j2][j].second = interval.first;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][j].first;
                    new1.second = interval.first;
                    new2.first = interval.second;
                    new2.second = safe_intervals[i2][j2][j].second;
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, new2);
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, new1);
                }
            }
            else if(safe_intervals[i2][j2][j].first >= interval.first && safe_intervals[i2][j2][j].first < interval.second)
            {
                if(safe_intervals[i2][j2][j].first == interval.first)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, {safe_intervals[i2][j2][j].first,safe_intervals[i2][j2][j].first});
                    j++;
                }
                if(safe_intervals[i2][j2][j].second < interval.second)
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                else
                    safe_intervals[i2][j2][j].first = interval.second;
            }
        }
    }
}

std::vector<std::pair<double, double> > Constraints::getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int w)
{
    std::vector<std::pair<double, double> > intervals(0);
    auto range = close.equal_range(curNode.i*w + curNode.j);
    for(unsigned int i = 0; i < safe_intervals[curNode.i][curNode.j].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][i].second >= curNode.g
                && safe_intervals[curNode.i][curNode.j][i].first <= (curNode.Parent->interval.second + curNode.g - curNode.Parent->g))
        {
            bool has = false;
            for(auto it = range.first; it != range.second; it++)
                if(it->second.interval.first == safe_intervals[curNode.i][curNode.j][i].first)
                {
                    has = true;
                    break;
                }
            if(!has)
                intervals.push_back(safe_intervals[curNode.i][curNode.j][i]);
        }
    return intervals;
}

std::vector<std::pair<double, double> > Constraints::getSafeIntervals(Node curNode)
{
    return safe_intervals[curNode.i][curNode.j];
}

VelocityConstraints::VelocityConstraints(int width, int height):Constraints(width, height)
{
    constraints.resize(height);
    for(int i = 0; i < height; i++)
    {
        constraints[i].resize(width);
        for(int j = 0; j < width; j++)
            constraints[i][j].resize(0);
    }
}

void VelocityConstraints::addConstraints(const std::vector<Node> &sections, double size)
{
    std::vector<std::pair<int,int>> cells;
    LineOfSight los(size);
    section sec(sections.back(), sections.back());
    sec.g2 = CN_INFINITY;
    sec.size = size;

    cells = los.getCellsCrossedByLine(sec.i1, sec.j1, sec.i2, sec.j2);
    for(auto cell: cells)
        constraints[cell.first][cell.second].push_back(sec);
    if(sections.size() == 1)
        for(auto cell: cells)
            safe_intervals[cell.first][cell.second].clear();
    for(unsigned int a = 1; a < sections.size(); a++)
    {
        cells = los.getCellsCrossedByLine(sections[a-1].i, sections[a-1].j, sections[a].i, sections[a].j);
        sec = section(sections[a-1], sections[a]);
        sec.size = size;
        for(unsigned int i = 0; i < cells.size(); i++)
            constraints[cells[i].first][cells[i].second].push_back(sec);
        if(a+1 == sections.size())
            updateSafeIntervals(cells,sec,true);
        else
            updateSafeIntervals(cells,sec,false);
    }
}

std::vector<std::pair<double,double>> VelocityConstraints::findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w)
{
    std::vector<std::pair<double,double>> curNodeIntervals = getSafeIntervals(curNode, close, w);
    if(curNodeIntervals.empty())
        return curNodeIntervals;
    EAT.clear();

    LineOfSight los(agentsize);
    std::vector<std::pair<int,int>> cells = los.getCellsCrossedByLine(curNode.i, curNode.j, curNode.Parent->i, curNode.Parent->j);
    std::vector<section> sections(0);
    section sec;
    for(unsigned int i = 0; i < cells.size(); i++)
        for(unsigned int j = 0; j < constraints[cells[i].first][cells[i].second].size(); j++)
        {
            sec = constraints[cells[i].first][cells[i].second][j];
            if(sec.g2 < curNode.Parent->g || sec.g1 > (curNode.Parent->interval.second + curNode.g - curNode.Parent->g))
                continue;
            if(std::find(sections.begin(), sections.end(), sec) == sections.end())
                sections.push_back(sec);
        }
    auto range = close.equal_range(curNode.i*w + curNode.j);
    for(unsigned int i=0; i<curNodeIntervals.size(); i++)
    {
        std::pair<double,double> cur_interval(curNodeIntervals[i]);
        if(cur_interval.first < curNode.g)
            cur_interval.first = curNode.g;
        double startTimeA = curNode.Parent->g;
        if(cur_interval.first > startTimeA + curNode.g - curNode.Parent->g)
            startTimeA = cur_interval.first - curNode.g + curNode.Parent->g;
        unsigned int j = 0;
        bool goal_collision;
        while(j < sections.size())
        {
            goal_collision = false;
            if(hasCollision(curNode, startTimeA, sections[j], goal_collision))
            {
                startTimeA += 0.1;
                cur_interval.first += 0.1;
                j = 0;//start to check all constraints again, because time has changed
                if(goal_collision || cur_interval.first > cur_interval.second || startTimeA > curNode.Parent->interval.second)
                {
                    curNodeIntervals.erase(curNodeIntervals.begin() + i);
                    i--;
                    break;
                }
            }
            else
                j++;
        }
        if(j == sections.size())
        {
            bool has=false;
            for(auto rit = range.first; rit != range.second; rit++)
                if(rit->second.interval.first == curNodeIntervals[i].first)
                    {
                        has = true;
                        curNodeIntervals.erase(curNodeIntervals.begin()+i);
                        i--;
                        break;
                    }
            if(!has)
                EAT.push_back(cur_interval.first);
        }
    }
    return curNodeIntervals;
}

bool VelocityConstraints::hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision)
{
    double endTimeA(startTimeA + curNode.g - curNode.Parent->g), startTimeB(constraint.g1), endTimeB(constraint.g2);
    if(startTimeA > endTimeB || startTimeB > endTimeA)
        return false;
    Vector2D A(curNode.Parent->i,curNode.Parent->j);
    Vector2D VA((curNode.i - curNode.Parent->i)/(curNode.g - curNode.Parent->g), (curNode.j - curNode.Parent->j)/(curNode.g - curNode.Parent->g));
    Vector2D B(constraint.i1, constraint.j1);
    Vector2D VB((constraint.i2 - constraint.i1)/(constraint.g2 - constraint.g1), (constraint.j2 - constraint.j1)/(constraint.g2 - constraint.g1));
    if(startTimeB > startTimeA)
    {
      // Move A to the same time instant as B
      A += VA*(startTimeB-startTimeA);
      startTimeA=startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
      B += VB*(startTimeA - startTimeB);
      startTimeB = startTimeA;
    }
    double r(constraint.size + agentsize); // Combined radius
    Vector2D w(B - A);
    double c(w*w - r*r);
    if(c < 0)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    } // Agents are currently colliding

    // Use the quadratic formula to detect nearest collision (if any)
    Vector2D v(VA - VB);
    double a(v*v);
    double b(w*v);

    double dscr(b*b - a*c);
    if(dscr <= 0)
    {
        return false;
    }

    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    }
    else
    {
        return false;
    }
}
