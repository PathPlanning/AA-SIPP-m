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
}

bool sort_function(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}

std::vector<std::pair<int,int>> Constraints::findConflictCells(Node cur)
{
    std::vector<std::pair<int,int>> cells(0);
    int i1 = cur.i, j1 = cur.j, i2 = cur.Parent->i, j2 = cur.Parent->j;
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    int sep_value = delta_i*delta_i + delta_j*delta_j;
    if((delta_i + delta_j) == 0)//this situation is possible after modification of hppath and is needed for addConstraints function
        cells.push_back({i,j});
    else if(delta_i == 0)
        for(; j != j2+step_j; j += step_j)
            cells.push_back({i,j});
    else if(delta_j == 0)
        for(; i != i2+step_i; i += step_i)
            cells.push_back({i,j});
    else if(delta_i > delta_j)
    {
        for(; i != i2; i += step_i)
        {
            cells.push_back({i,j});
            cells.push_back({i,j+step_j});
            error += delta_j;
            if(error > delta_i)
            {
                j += step_j;
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    cells.push_back({i + step_i,j - step_j});
                if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                    cells.push_back({i,j + step_j});
                error -= delta_i;
            }
        }
        cells.push_back({i,j});
        cells.push_back({i,j+step_j});
    }
    else
    {
        for(; j != j2; j += step_j)
        {
            cells.push_back({i,j});
            cells.push_back({i+step_i,j});
            error += delta_i;
            if(error > delta_j)
            {
                i += step_i;
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    cells.push_back({i-step_i,j+step_j});
                if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < sep_value)
                    cells.push_back({i+step_i,j});
                error -= delta_j;
            }
        }
        cells.push_back({i,j});
        cells.push_back({i+step_i,j});
    }
    return cells;
}


PointConstraints::PointConstraints(int width, int height):Constraints(width, height)
{
    gap = 2.0;
    constraints.resize(height);
    for(int i = 0; i < height; i++)
    {
        constraints[i].resize(width);
        for(int j = 0; j < width; j++)
            constraints[i][j].resize(0);
    }
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

void VelocityConstraints::addConstraints(const std::vector<Node> &sections)
{
    Node cur;
    std::vector<std::pair<int,int>> cells;
    if(sections.size() == 1)
    {
        section sec(sections.back(),sections.back());
        sec.g2 = CN_INFINITY;
        constraints[sections.back().i][sections.back().j].push_back(sec);
        return;
    }
    for(int a = 1; a < sections.size(); a++)
    {
        cur = sections[a];
        cells = findConflictCells(cur);
        int i1(sections[a].i), j1(sections[a].j), i0(sections[a-1].i), j0(sections[a-1].j), i2, j2;
        section sec(*cur.Parent, cur);
        for(int i = 0; i < cells.size(); i++)
            constraints[cells[i].first][cells[i].second].push_back(sec);
        if(a + 1 == sections.size())
        {
            section seg_goal(sections.back(), sections.back());
            seg_goal.g2 = CN_INFINITY;
            constraints[seg_goal.i1][seg_goal.j1].push_back(seg_goal);
        }
        for(int i = 0; i < cells.size(); i++)
        {
            i2 = cells[i].first;
            j2 = cells[i].second;
            std::pair<double,double> ps, pg, interval;
            ps = {i0, j0};
            pg = {i1, j1};
            double dist = fabs((ps.first - pg.first)*j2 + (pg.second - ps.second)*i2 + (ps.second*pg.first - ps.first*pg.second))
                    /sqrt(pow(ps.first - pg.first, 2) + pow(ps.second - pg.second, 2));
            double da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
            double db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
            double ha = sqrt(da - dist*dist);
            double hb = sqrt(db - dist*dist);
            double size = sqrt(1.0 - dist*dist);
            if(da == 0 && db == 0)
            {
                interval.first = cur.Parent->g;
                interval.second = cur.g;
            }
            else if(da >= 1.0 && db >= 1.0)
            {
                interval.first = cur.Parent->g + ha - size;
                interval.second = cur.Parent->g + ha + size;
            }
            else if(da < 1.0)
            {
                interval.first = cur.Parent->g;
                interval.second = cur.g - hb + size;
            }
            else
            {
                interval.first = cur.Parent->g + ha - size;
                interval.second = cur.g;
            }
            for(int j = 0; j < safe_intervals[i2][j2].size(); j++)
            {
                if(safe_intervals[i2][j2][j].first <= interval.first && safe_intervals[i2][j2][j].second >= interval.first)
                {
                    if(safe_intervals[i2][j2][j].first == interval.first)
                    {
                        if(safe_intervals[i2][j2][j].second <= interval.second)
                            safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                        else
                            safe_intervals[i2][j2][j].first = interval.second;
                        break;
                    }
                    else if(safe_intervals[i2][j2][j].second <= interval.second)
                    {
                        safe_intervals[i2][j2][j].second = interval.first;
                        break;
                    }
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
                        break;
                    }
                }
                else if(safe_intervals[i2][j2][j].first >= interval.first && safe_intervals[i2][j2][j].first < interval.second)
                {
                    if(safe_intervals[i2][j2][j].second <= interval.second)
                    {
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                        break;
                    }
                    else
                    {
                        safe_intervals[i2][j2][j].first = interval.second;
                        break;
                    }
                }
            }
        }
    }
}

std::vector<std::pair<double,double>> VelocityConstraints::findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w)
{
    std::vector<std::pair<double,double>> curNodeIntervals(0);
    double ab = curNode.g - curNode.Parent->g;
    auto range = close.equal_range(curNode.i * w + curNode.j);
    bool has;
    for(int i = 0; i < safe_intervals[curNode.i][curNode.j].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][i].second > curNode.g && safe_intervals[curNode.i][curNode.j][i].first <= curNode.Parent->interval.second + ab)
        {
            has = false;
            for(auto it = range.first; it != range.second; it++)
                if(it->second.interval.first == safe_intervals[curNode.i][curNode.j][i].first)
                {
                    has = true;
                    break;
                }
            if(!has)
            {
                curNodeIntervals.push_back(safe_intervals[curNode.i][curNode.j][i]);
                if(curNodeIntervals.back().first < curNode.g)
                    EAT.push_back(curNode.g);
                else
                    EAT.push_back(curNodeIntervals.back().first);
            }
        }
    if(curNodeIntervals.empty())
        return curNodeIntervals;

    std::vector<std::pair<int,int>> cells = findConflictCells(curNode);
    std::vector<section> sections(0);
    section con;
    for(int i = 0; i < cells.size(); i++)
        for(int j=0; j<constraints[cells[i].first][cells[i].second].size(); j++)
        {
            con = constraints[cells[i].first][cells[i].second][j];
            if(con.g2 < curNode.Parent->g || con.g1 > curNode.Parent->interval.second+ab)
                continue;
            if(std::find(sections.begin(), sections.end(), con) == sections.end())
                sections.push_back(con);
        }
    for(int i=0; i<curNodeIntervals.size(); i++)
    {
        std::pair<double,double> cur_interval(curNodeIntervals[i]);
        if(cur_interval.first < curNode.g)
            cur_interval.first = curNode.g;
        double startTimeA = curNode.Parent->g;
        if(cur_interval.first > startTimeA+ab)
            startTimeA = cur_interval.first-ab;
        int j = 0;
        bool goal_collision;
        while(j < sections.size())
        {
            goal_collision = false;
            if(hasCollision(curNode, startTimeA, sections[j], goal_collision))
            {
                startTimeA += 1.0;
                cur_interval.first += 1.0;
                j = 0;//start to check all constraints again, because time has changed
                if(goal_collision || cur_interval.first > cur_interval.second || startTimeA > curNode.Parent->interval.second)
                {
                    curNodeIntervals.erase(curNodeIntervals.begin() + i);
                    EAT.erase(EAT.begin() + i);
                    i--;
                    break;
                }
            }
            else
                j++;
        }
        if(j == sections.size())
            EAT[i] = cur_interval.first;
    }
    return curNodeIntervals;
}

bool VelocityConstraints::hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision)
{
    double endTimeA(startTimeA + curNode.g - curNode.Parent->g), startTimeB(constraint.g1), endTimeB(constraint.g2);
    if(fgreater(startTimeA,endTimeB) || fgreater(startTimeB,endTimeA))
        return false;
    Vector2D A(curNode.Parent->i,curNode.Parent->j);
    Vector2D VA((curNode.i - curNode.Parent->i)/(curNode.g - curNode.Parent->g), (curNode.j - curNode.Parent->j)/(curNode.g - curNode.Parent->g));
    Vector2D B(constraint.i1, constraint.j1);
    Vector2D VB((constraint.i2 - constraint.i1)/(constraint.g2 - constraint.g1), (constraint.j2 - constraint.j1)/(constraint.g2 - constraint.g1));
    if(fgreater(startTimeB,startTimeA))
    {
      // Move A to the same time instant as B
      A+=VA*(startTimeB-startTimeA);
      startTimeA=startTimeB;
    }
    else if(fless(startTimeB,startTimeA))
    {
      B+=VB*(startTimeA-startTimeB);
      startTimeB=startTimeA;
    }

    double r(1.0); // Combined radius
    Vector2D w(B-A);
    double c(w.sq() - r*r);
    if(c < 0)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    } // Agents are currently colliding

    // Use the quadratic formula to detect nearest collision (if any)
    Vector2D v(VA-VB);
    double a(v.sq());
    double b(w*v);

    double dscr(b*b - a*c);
    if(fleq(dscr,0))
        return false;

    double ctime = (b - sqrt(dscr))/a;
    if(fgeq(ctime,0) && fleq(ctime, min(endTimeB,endTimeA) - startTimeA))
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    }
    else
        return false;
}

void PointConstraints::addConstraints(const std::vector<Node> &sections)
{
    Node cur;
    std::vector<std::pair<int,int>> cells;
    if(sections.size() == 1)
    {
        constraint add;
        add.i = sections.back().i;
        add.j = sections.back().j;
        add.g = 0;
        add.goal = true;
        constraints[add.i][add.j].push_back(add);
        return;
    }
    for(int a = 1; a < sections.size(); a++)
    {
        cur = sections[a];
        cells = findConflictCells(cur);
        int x1 = cur.i, y1 = cur.j, x0 = cur.Parent->i, y0 = cur.Parent->j;
        constraint add;
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        if(dx == 0 && dy == 0)
        {
            add.i = cur.i;
            add.j = cur.j;
            add.goal = false;
            for(double i = cur.Parent->g; i <= cur.g; i += gap)
            {
                add.g = i;
                if(constraints[add.i][add.j].empty() || constraints[add.i][add.j].back().g != add.g)
                    constraints[add.i][add.j].push_back(add);
            }
            if(constraints[add.i][add.j].back().g < cur.g)
            {
                add.g = cur.g;
                constraints[add.i][add.j].push_back(add);
            }
            continue;
        }
        else
        {
            for(int i = 0; i < cells.size(); i++)
            {
                add.i = cells[i].first;
                add.j = cells[i].second;
                std::pair<double,double> ps,pg;
                ps={x0, y0};
                pg={x1, y1};
                double dist = fabs((ps.first - pg.first)*add.j + (pg.second - ps.second)*add.i + (ps.second*pg.first - ps.first*pg.second))
                        /sqrt(pow(ps.first - pg.first,2) + pow(ps.second - pg.second,2));
                double da = (x0 - add.i)*(x0 - add.i) + (y0 - add.j)*(y0 - add.j);
                double db = (x1 - add.i)*(x1 - add.i) + (y1 - add.j)*(y1 - add.j);
                double ha = sqrt(da - dist*dist);
                double hb = sqrt(db - dist*dist);
                constraint con;

                if(hb > 0)
                {
                    double lambda = ha/hb;
                    con.i = (x0 + lambda*x1)/(1 + lambda);
                    con.j = (y0 + lambda*y1)/(1 + lambda);
                }
                else
                {
                    con.i = x1;
                    con.j = y1;
                }
                con.g = cur.Parent->g + sqrt(pow(cur.Parent->i - con.i, 2) + pow(cur.Parent->j - con.j, 2));
                if(add.i == sections.back().i && add.j == sections.back().j)
                    con.goal = true;
                else
                    con.goal = false;
                if(constraints[add.i][add.j].empty() || fabs(constraints[add.i][add.j].back().g-con.g)>CN_EPSILON)
                    constraints[add.i][add.j].push_back(con);
            }
        }
        for(int i = 0; i < cells.size(); i++)
        {
            add.i = cells[i].first;
            add.j = cells[i].second;
            std::pair<double,double> ps, pg, interval;
            ps = {x0, y0};
            pg = {x1, y1};
            double dist = fabs((ps.first - pg.first)*add.j + (pg.second - ps.second)*add.i + (ps.second*pg.first - ps.first*pg.second))
                    /sqrt(pow(ps.first - pg.first,2) + pow(ps.second - pg.second,2));
            double da = (x0 - add.i)*(x0 - add.i) + (y0 - add.j)*(y0 - add.j);
            double db = (x1 - add.i)*(x1 - add.i) + (y1 - add.j)*(y1 - add.j);
            double ha = sqrt(da - dist*dist);
            double hb = sqrt(db - dist*dist);
            double size = sqrt(1 - dist*dist);
            if(da == 0 && db == 0)
            {
                interval.first = cur.Parent->g;
                interval.second = cur.g;
            }
            if(da >= 1 && db >= 1)
            {
                interval.first = cur.Parent->g + ha - size;
                interval.second = cur.Parent->g + ha + size;
            }
            else if(da < 1)
            {
                interval.first = cur.Parent->g;
                interval.second = cur.g - hb + size;
            }
            else
            {
                interval.first = cur.Parent->g + ha - size;
                interval.second = cur.g;
            }
            for(int j = 0; j < safe_intervals[add.i][add.j].size(); j++)
            {
                if(safe_intervals[add.i][add.j][j].first <= interval.first && safe_intervals[add.i][add.j][j].second >= interval.first)
                {
                    if(fabs(safe_intervals[add.i][add.j][j].first - interval.first) < CN_EPSILON)
                    {
                        if(safe_intervals[add.i][add.j][j].second <= interval.second)
                            safe_intervals[add.i][add.j].erase(safe_intervals[add.i][add.j].begin() + j);
                        else
                            safe_intervals[add.i][add.j][j].first = interval.second;
                        break;
                    }
                    else if(safe_intervals[add.i][add.j][j].second <= interval.second)
                    {
                        safe_intervals[add.i][add.j][j].second = interval.first;
                        break;
                    }
                    else
                    {
                        std::pair<double,double> new1, new2;
                        new1.first = safe_intervals[add.i][add.j][j].first;
                        new1.second = interval.first;
                        new2.first = interval.second;
                        new2.second = safe_intervals[add.i][add.j][j].second;
                        safe_intervals[add.i][add.j].erase(safe_intervals[add.i][add.j].begin() + j);
                        safe_intervals[add.i][add.j].insert(safe_intervals[add.i][add.j].begin() + j, new2);
                        safe_intervals[add.i][add.j].insert(safe_intervals[add.i][add.j].begin() + j, new1);
                        break;
                    }
                }
                else if(safe_intervals[add.i][add.j][j].first >= interval.first && safe_intervals[add.i][add.j][j].first < interval.second)
                {
                    if(safe_intervals[add.i][add.j][j].second <= interval.second)
                    {
                        safe_intervals[add.i][add.j].erase(safe_intervals[add.i][add.j].begin() + j);
                        break;
                    }
                    else
                    {
                        safe_intervals[add.i][add.j][j].first = interval.second;
                        break;
                    }
                }
            }
        }
    }
}


std::vector<std::pair<double,double>> PointConstraints::findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w)
{
    std::vector<std::pair<double,double>> badIntervals(0), curNodeIntervals(0);
    double ab = curNode.g-curNode.Parent->g;

    auto range = close.equal_range(curNode.i * w + curNode.j);
    bool has;
    for(int i = 0; i < safe_intervals[curNode.i][curNode.j].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][i].second > curNode.g && safe_intervals[curNode.i][curNode.j][i].first <= curNode.Parent->interval.second + ab)
        {
            has = false;
            for(auto it = range.first; it != range.second; it++)
                if(it->second.interval.first == safe_intervals[curNode.i][curNode.j][i].first)
                {
                    has = true;
                    break;
                }
            if(!has)
            {
                curNodeIntervals.push_back(safe_intervals[curNode.i][curNode.j][i]);
                if(curNodeIntervals.back().first < curNode.g)
                    EAT.push_back(curNode.g);
                else
                    EAT.push_back(curNodeIntervals.back().first);
            }
        }
    if(curNodeIntervals.empty())
        return curNodeIntervals;
    std::vector<std::pair<int,int>> cells = findConflictCells(curNode);

    double da, db, offset, vi, vj, wi, wj, c1, c2, dist;
    std::pair<double,double> add;
    constraint con;
    for(int i = 0; i < cells.size(); i++)
    {
        for(int j = 0; j < constraints[cells[i].first][cells[i].second].size(); j++)
        {

            con = constraints[cells[i].first][cells[i].second][j];
            if(con.g + gap < curNode.Parent->g && !con.goal)
                continue;
            da = (curNode.i - con.i)*(curNode.i - con.i) + (curNode.j - con.j)*(curNode.j - con.j);
            db = (curNode.Parent->i - con.i)*(curNode.Parent->i - con.i) + (curNode.Parent->j - con.j)*(curNode.Parent->j - con.j);
            vi = curNode.Parent->i - curNode.i;
            vj = curNode.Parent->j - curNode.j;
            wi = con.i - curNode.i;
            wj = con.j - curNode.j;
            c1 = vj*wj + vi*wi;
            c2 = vj*vj + vi*vi;
            if(c1 <= 0 || c2 <= c1)//if constraint is outside of the section
            {
                if(da < db)
                    dist = da;
                else
                    dist = db;
                if(dist < 1)//less than 2r
                {
                    if(dist == da)
                        add={con.g - gap, con.g + gap};
                    else
                        add={con.g - gap + ab, con.g + gap + ab};
                    if(con.goal == true)
                        add.second = CN_INFINITY;
                    badIntervals.push_back(add);
                    continue;
                }

            }
            else
            {
                dist = fabs(((curNode.Parent->i - curNode.i)*con.j + (curNode.j - curNode.Parent->j)*con.i
                             +(curNode.Parent->j*curNode.i - curNode.Parent->i*curNode.j)))/ab;
                if(dist < 1)
                {
                    offset = sqrt(da - dist*dist);
                    add = {con.g - gap + offset, con.g + gap + offset};
                    if(con.goal == true)
                        add.second = CN_INFINITY;
                    badIntervals.push_back(add);
                }
            }
        }
    }

    //combining and sorting bad intervals
    if(badIntervals.size() > 1)
    {
        std::sort(badIntervals.begin(), badIntervals.end(), sort_function);
        std::pair<double,double> cur, next;
        for(int i = 0; i < badIntervals.size() - 1; i++)
        {
            cur = badIntervals[i];
            next = badIntervals[i + 1];
            if((cur.first - next.first)*(cur.second - next.first) < CN_EPSILON)
            {
                if(next.second > cur.second)
                    badIntervals[i].second = next.second;
                badIntervals.erase(badIntervals.begin() + i + 1);
                i--;
            }
        }
    }

    //searching reachebale intervals and theirs EAT
    if(badIntervals.size() > 0)
    {
        for(int i = 0; i < badIntervals.size(); i++)
            for(int j = 0; j < curNodeIntervals.size(); j++)
                if(badIntervals[i].first <= EAT[j])
                {
                    if(badIntervals[i].second >= curNodeIntervals[j].second)
                    {
                        curNodeIntervals.erase(curNodeIntervals.begin() + j);
                        EAT.erase(EAT.begin() + j);
                        j--;
                        continue;
                    }
                    else if(badIntervals[i].second > EAT[j])
                        EAT[j] = badIntervals[i].second;
                }
        for(int i = 0; i < curNodeIntervals.size(); i++)
            if(EAT[i] > curNode.Parent->interval.second + ab || curNodeIntervals[i].second < curNode.g)
            {
                curNodeIntervals.erase(curNodeIntervals.begin() + i);
                EAT.erase(EAT.begin() + i);
                i--;
            }
    }
    return curNodeIntervals;
}
