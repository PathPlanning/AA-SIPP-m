#include "constraints.h"

Constraints::Constraints(int size)
{
    safe_intervals.resize(size, {0,CN_INFINITY});
    constraints.clear();
}

bool sort_function(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}

double Constraints::minDist(Point A, Point C, Point D)
{
    int classA = A.classify(C, D);
    if(classA == 3)
        return sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2));
    else if(classA == 4)
        return sqrt(pow(A.i - D.i, 2) + pow(A.j - D.j, 2));
    else
        return fabs((C.i - D.i)*A.j + (D.j - C.j)*A.i + (C.j*D.i - D.j*C.i))/sqrt(pow(C.i - D.i, 2) + pow(C.j - D.j, 2));
}

void Constraints::resetSafeIntervals(int size)
{
    safe_intervals.resize(size);
    for(auto &si:safe_intervals)
    {
        si.clear();
        si.push_back(SafeInterval(0, CN_INFINITY, 0));
    }
}

void Constraints::updateCellSafeIntervals(Node n, const Map& map)
{
    if(safe_intervals[n.id].size() > 1)
        return;
    for(int k = 0; k < constraints.size(); k++)
    {
        section sec = constraints[k];
        double radius = agentsize + sec.size;
        double i0(constraints[k].i1), j0(constraints[k].j1), i1(constraints[k].i2), j1(constraints[k].j2), i2(n.i), j2(n.j);
        SafeInterval interval;
        double dist, mindist;
        if(fabs(i0 - i1) < CN_EPSILON && fabs(j0 - j1) < CN_EPSILON && fabs(i0 - i2) < CN_EPSILON && fabs(j0 - j2) < CN_EPSILON)
            mindist = 0;
        else
            mindist = minDist(Point(i2,j2), Point(i0,j0), Point(i1,j1));
        if(mindist >= radius)
            continue;
        Point point(i2,j2), p0(i0,j0), p1(i1,j1);
        int cls = point.classify(p0, p1);
        dist = fabs((i0 - i1)*j2 + (j1 - j0)*i2 + (j0*i1 - i0*j1))/sqrt(pow(i0 - i1, 2) + pow(j0 - j1, 2));
        double da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
        double db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
        double ha = sqrt(da - dist*dist);
        double size = sqrt(radius*radius - dist*dist);
        if(cls == 3)
        {
            interval.begin = sec.g1;
            interval.end = sec.g1 + (sqrt(radius*radius - dist*dist) - ha)/sec.mspeed;
        }
        else if(cls == 4)
        {
            interval.begin = sec.g2 - sqrt(radius*radius - dist*dist)/sec.mspeed + sqrt(db - dist*dist)/sec.mspeed;
            interval.end = sec.g2;
        }
        else if(da < radius*radius)
        {
            if(db < radius*radius)
            {
                interval.begin = sec.g1;
                interval.end = sec.g2;
            }
            else
            {
                double hb = sqrt(db - dist*dist);
                interval.begin = sec.g1;
                interval.end = sec.g2 - hb/sec.mspeed + size/sec.mspeed;
            }
        }
        else
        {
            if(db < radius*radius)
            {
                interval.begin = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.end = sec.g2;
            }
            else
            {
                interval.begin = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.end = sec.g1 + ha/sec.mspeed + size/sec.mspeed;
            }
        }
        for(unsigned int j = 0; j < safe_intervals[n.id].size(); j++)
        {
            if(safe_intervals[n.id][j].begin < interval.begin + CN_EPSILON && safe_intervals[n.id][j].end + CN_EPSILON > interval.begin)
            {
                if(fabs(safe_intervals[n.id][j].begin - interval.begin) < CN_EPSILON)
                {
                    safe_intervals[n.id].insert(safe_intervals[n.id].begin() + j, SafeInterval(safe_intervals[n.id][j].begin,safe_intervals[n.id][j].begin));
                    j++;
                    if(safe_intervals[n.id][j].end < interval.end)
                        safe_intervals[n.id].erase(safe_intervals[n.id].begin() + j);
                    else
                        safe_intervals[n.id][j].begin = interval.end;
                }
                else if(safe_intervals[n.id][j].end < interval.end)
                    safe_intervals[n.id][j].end = interval.begin;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[n.id][j].begin;
                    new1.second = interval.begin;
                    new2.first = interval.end;
                    new2.second = safe_intervals[n.id][j].end;
                    safe_intervals[n.id].erase(safe_intervals[n.id].begin() + j);
                    if(new2.first < CN_INFINITY)
                        safe_intervals[n.id].insert(safe_intervals[n.id].begin() + j, SafeInterval(new2.first, new2.second));
                    safe_intervals[n.id].insert(safe_intervals[n.id].begin() + j, SafeInterval(new1.first, new1.second));
                }
            }
            else if(safe_intervals[n.id][j].begin > interval.begin - CN_EPSILON && safe_intervals[n.id][j].begin < interval.end)
            {
                if(fabs(safe_intervals[n.id][j].begin - interval.begin) < CN_EPSILON)
                {
                    safe_intervals[n.id].insert(safe_intervals[n.id].begin() + j, SafeInterval(safe_intervals[n.id][j].begin,safe_intervals[n.id][j].begin));
                    j++;
                }
                if(safe_intervals[n.id][j].end < interval.end)
                {
                    safe_intervals[n.id].erase(safe_intervals[n.id].begin() + j);
                }
                else
                {
                    safe_intervals[n.id][j].begin = interval.end;
                }
            }
        }
        for(unsigned int j = 0; j < safe_intervals[n.id].size(); j++)
            safe_intervals[n.id][j].id = j;
    }
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int w)
{
    std::vector<SafeInterval> intervals(0);
    auto range = close.equal_range(curNode.id);
    for(unsigned int i = 0; i < safe_intervals[curNode.id].size(); i++)
        if(safe_intervals[curNode.id][i].end >= curNode.g
                && safe_intervals[curNode.id][i].begin <= (curNode.Parent->interval.end + curNode.g - curNode.Parent->g))
        {
            bool has = false;
            for(auto it = range.first; it != range.second; it++)
                if(it->second.interval.id == safe_intervals[curNode.id][i].id)
                if((it->second.g + tweight*fabs(curNode.heading - it->second.heading)/(180*rspeed)) - curNode.g < CN_EPSILON)//take into account turning cost
                {
                    has = true;
                    break;
                }
            if(!has)
                intervals.push_back(safe_intervals[curNode.id][i]);
        }
    return intervals;
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode)
{
    return safe_intervals[curNode.id];
}

void Constraints::addStartConstraint(int id, double i, double j, int size, double agentsize)
{
    section sec(id, id, 0, size);
    sec.set_ij(i,j,i,j);
    sec.size = agentsize;
    constraints.insert(constraints.begin(),sec);
    return;
}

void Constraints::removeStartConstraint(int id)
{
    for(size_t k = 0; k < constraints.size(); k++)
        if(constraints[k].id1 == id && constraints[k].g1 < CN_EPSILON)
        {
            constraints.erase(constraints.begin() + k);
            k--;
        }
    return;
}

void Constraints::addConstraints(const std::vector<Node> &sections, double size, double mspeed, const Map &map)
{
    section sec(sections.back(), sections.back());
    sec.g2 = CN_INFINITY;
    sec.size = size;
    sec.mspeed = mspeed;
    constraints.push_back(sec);
    if(sec.g1 == 0)
        safe_intervals[sec.id1].clear();
    for(unsigned int a = 1; a < sections.size(); a++)
    {
        sec = section(sections[a-1], sections[a]);
        sec.size = size;
        sec.mspeed = mspeed;
        constraints.push_back(sec);
    }
}

std::vector<SafeInterval> Constraints::findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, const Map &map)
{
    std::vector<SafeInterval> curNodeIntervals = getSafeIntervals(curNode, close, map.width);
    if(curNodeIntervals.empty())
        return curNodeIntervals;
    EAT.clear();
    std::vector<section> sections(0);
    section sec;
    for(unsigned int j = 0; j < constraints.size(); j++)
    {
        sec = constraints[j];
        //if(sec.g2 < curNode.Parent->g || sec.g1 > (curNode.Parent->interval.end + curNode.g - curNode.Parent->g))
        //    continue;
        //if(sqrt(pow(sec.i1 - curNode.Parent->i,2) + pow(sec.j1 - curNode.Parent->j,2)) > (curNode.g - curNode.Parent->g + sec.g2 - sec.g1))
        //    continue;
        sections.push_back(sec);
    }
    auto range = close.equal_range(curNode.id);

    for(unsigned int i=0; i<curNodeIntervals.size(); i++)
    {
        SafeInterval cur_interval(curNodeIntervals[i]);
        if(cur_interval.begin < curNode.g)
            cur_interval.begin = curNode.g;
        double startTimeA = curNode.Parent->g;
        if(cur_interval.begin > startTimeA + curNode.g - curNode.Parent->g)
            startTimeA = cur_interval.begin - curNode.g + curNode.Parent->g;
        unsigned int j = 0;
        bool goal_collision;
        while(j < sections.size())
        {
            goal_collision = false;

            if(hasCollision(curNode, startTimeA, sections[j], goal_collision))
            {
                double offset = 1.0;
                startTimeA += offset;
                cur_interval.begin += offset;
                j = 0;//start to check all constraints again, because time has changed
                if(goal_collision || cur_interval.begin > cur_interval.end || startTimeA > curNode.Parent->interval.end)
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
            bool has = false;
            for(auto rit = range.first; rit != range.second; rit++)
                if(rit->second.interval.id == curNodeIntervals[i].id)
                if((rit->second.g + tweight*fabs(curNode.heading - rit->second.heading)/(180*rspeed) - cur_interval.begin) < CN_EPSILON)//take into account turning cost
                {
                    has = true;
                    curNodeIntervals.erase(curNodeIntervals.begin()+i);
                    i--;
                    break;
                }
            if(!has)
                EAT.push_back(cur_interval.begin);
        }
    }
    return curNodeIntervals;
}

bool Constraints::hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision)
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
    double r(constraint.size + agentsize + inflateintervals); //combined radius
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
        return false;

    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
    {
        if(fabs(constraint.g2 - CN_INFINITY) < CN_EPSILON)
            goal_collision = true;
        return true;
    }
    else
        return false;
}
