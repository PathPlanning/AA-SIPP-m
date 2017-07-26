#include "comboconstraints.h"
bool sort_f(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}
ComboConstraints::ComboConstraints(int width, int height):Constraints(width, height)
{
    gap = 2.0;
    s_constraints.resize(height);
    p_constraints.resize(height);
    for(int i = 0; i < height; i++)
    {
        s_constraints[i].resize(width);
        p_constraints[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            s_constraints[i][j].resize(0);
            p_constraints[i][j].resize(0);
        }
    }
}

int ComboConstraints::checkIntersection(Point A, Point B, Point C, Point D, Point &intersec)
{
    double denom  = (D.j - C.j)*(B.i - A.i) - (D.i - C.i)*(B.j - A.j);
    double nume_a = (D.i - C.i)*(A.j - C.j) - (D.j - C.j)*(A.i - C.i);
    double nume_b = (B.i - A.i)*(A.j - C.j) - (B.j - A.j)*(A.i - C.i);
    if(denom == 0.0)
    {
        if(nume_a == 0.0 && nume_b == 0.0)
            return CN_COINCIDENT;
        return CN_PARALLEL;
    }
    double ua = nume_a / denom;
    double ub = nume_b / denom;
    if(ua >= 0.0 && ua <= 1.0 && ub >= 0.0 && ub <= 1.0)
    {
        intersec = Point{A.i + ua*(B.i - A.i), A.j + ua*(B.j - A.j)};
        return CN_INTERSECTING;
    }
    return CN_NONINTERSECTING;
}

void ComboConstraints::addConstraints(const std::vector<Node> &sections)
{
    std::vector<std::pair<int,int>> cells;
    section sec(sections.back(), sections.back());
    sec.g2 = CN_INFINITY;
    s_constraints[sec.i1][sec.j1].push_back(sec);
    if(sections.size() == 1)
    {
        constraint add;
        add.i = sections.back().i;
        add.j = sections.back().j;
        add.g = 0;
        add.goal = true;
        p_constraints[add.i][add.j].push_back(add);
        safe_intervals[add.i][add.j].clear();
        return;
    }
    for(int a = 1; a < sections.size(); a++)
    {
        sec = section(sections[a-1], sections[a]);
        cells = findConflictCells(sections[a]);
        for(int i = 0; i < cells.size(); i++)
            s_constraints[cells[i].first][cells[i].second].push_back(sec);
        int x0 = sec.i1, y0 = sec.j1, x1 = sec.i2, y1 = sec.j2;
        constraint add;
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        if(dx == 0 && dy == 0)
        {
            add.i = sec.i1;
            add.j = sec.j1;
            add.goal = false;
            for(double i = sec.g1; i <= sec.g2; i += gap)
            {
                add.g = i;
                if(p_constraints[add.i][add.j].empty() || p_constraints[add.i][add.j].back().g != add.g)
                    p_constraints[add.i][add.j].push_back(add);
            }
            if(p_constraints[add.i][add.j].back().g < sec.g2)
            {
                add.g = sec.g2;
                p_constraints[add.i][add.j].push_back(add);
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
                con.g = sec.g1 + sqrt(pow(sec.i1 - con.i, 2) + pow(sec.j1 - con.j, 2));
                if(add.i == sections.back().i && add.j == sections.back().j)
                    con.goal = true;
                else
                    con.goal = false;
                if(p_constraints[add.i][add.j].empty() || fabs(p_constraints[add.i][add.j].back().g-con.g)>CN_EPSILON)
                    p_constraints[add.i][add.j].push_back(con);
            }
        }
        if(a+1 == sections.size())
            updateSafeIntervals(cells,sec,true);
        else
            updateSafeIntervals(cells,sec,false);
    }
}

std::vector<std::pair<double,double>> ComboConstraints::findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, int w)
{
    if(lastAgent == 0)//point constraints
    {
        std::vector<std::pair<double,double>> badIntervals(0), curNodeIntervals(getSafeIntervals(curNode,close,w));
        if(curNodeIntervals.empty())
            return curNodeIntervals;
        std::vector<std::pair<int,int>> cells = findConflictCells(curNode);
        double da, db, offset, c1, c2, dist;
        double ab = curNode.g - curNode.Parent->g;
        std::pair<double,double> add;
        constraint con;
        EAT.clear();
        for(int i=0; i<curNodeIntervals.size(); i++)
        {
            if(curNodeIntervals[i].first<curNode.g)
                EAT.push_back(curNode.g);
            else
                EAT.push_back(curNodeIntervals[i].first);
        }
        for(int i = 0; i < cells.size(); i++)
        {
            for(int j = 0; j < p_constraints[cells[i].first][cells[i].second].size(); j++)
            {
                con = p_constraints[cells[i].first][cells[i].second][j];
                if(con.g + gap < curNode.Parent->g && !con.goal)
                    continue;
                da = (curNode.i - con.i)*(curNode.i - con.i) + (curNode.j - con.j)*(curNode.j - con.j);
                c1 = (curNode.Parent->j - curNode.j)*(con.j - curNode.j) + (curNode.Parent->i - curNode.i)*(con.i - curNode.i);
                c2 = (curNode.Parent->j - curNode.j)*(curNode.Parent->j - curNode.j) + (curNode.Parent->i - curNode.i)*(curNode.Parent->i - curNode.i);
                if(c1 <= 0 || c2 <= c1)//if constraint is outside of the section
                {
                    db = (curNode.Parent->i - con.i)*(curNode.Parent->i - con.i) + (curNode.Parent->j - con.j)*(curNode.Parent->j - con.j);
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
                    if(dist < 1.0)
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
            std::sort(badIntervals.begin(), badIntervals.end(), sort_f);
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
    else if(lastAgent==1)//velocity constraints
    {
        std::vector<std::pair<double,double>> curNodeIntervals = getSafeIntervals(curNode, close, w);
        if(curNodeIntervals.empty())
            return curNodeIntervals;
        EAT.clear();
        std::vector<std::pair<int,int>> cells = findConflictCells(curNode);
        std::vector<section> sections(0);
        section sec;
        for(int i = 0; i < cells.size(); i++)
            for(int j=0; j<s_constraints[cells[i].first][cells[i].second].size(); j++)
            {
                sec = s_constraints[cells[i].first][cells[i].second][j];
                if(sec.g2 < curNode.Parent->g || sec.g1 > (curNode.Parent->interval.second + curNode.g - curNode.Parent->g))
                    continue;
                if(std::find(sections.begin(), sections.end(), sec) == sections.end())
                    sections.push_back(sec);
            }
        for(int i=0; i<curNodeIntervals.size(); i++)
        {
            std::pair<double,double> cur_interval(curNodeIntervals[i]);
            if(cur_interval.first < curNode.g)
                cur_interval.first = curNode.g;
            double startTimeA = curNode.Parent->g;
            if(cur_interval.first > startTimeA + curNode.g - curNode.Parent->g)
                startTimeA = cur_interval.first - curNode.g + curNode.Parent->g;
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
                        i--;
                        break;
                    }
                }
                else
                    j++;
            }
            if(j == sections.size())
                EAT.push_back(cur_interval.first);
        }
        return curNodeIntervals;
    }
    else//section constraints
    {
        std::vector<std::pair<double,double>> badIntervals(0), curNodeIntervals(getSafeIntervals(curNode,close,w));
        if(curNodeIntervals.empty())
            return curNodeIntervals;
        std::vector<std::pair<int,int>> cells = findConflictCells(curNode);
        EAT.clear();
        for(int i=0; i<curNodeIntervals.size(); i++)
        {
            if(curNodeIntervals[i].first<curNode.g)
                EAT.push_back(curNode.g);
            else
                EAT.push_back(curNodeIntervals[i].first);
        }
        std::vector<section> sections(0);
        section sec;
        for(int i = 0; i < cells.size(); i++)
            for(int j=0; j<s_constraints[cells[i].first][cells[i].second].size(); j++)
            {
                sec = s_constraints[cells[i].first][cells[i].second][j];
                if(sec.g2 < curNode.Parent->g || sec.g1 > (curNode.Parent->interval.second + curNode.g - curNode.Parent->g))
                    continue;
                if(std::find(sections.begin(), sections.end(), sec) == sections.end())
                    sections.push_back(sec);
            }
        for(int i=0; i<sections.size(); i++)
        {
            std::pair<double, double> badInterval = countInterval(sections[i], curNode);
            if(badInterval.second > 0)
                badIntervals.push_back(badInterval);
        }
        //combining and sorting bad intervals
        if(badIntervals.size() > 1)
        {
            std::sort(badIntervals.begin(), badIntervals.end(), sort_f);
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
                if(EAT[i] > curNode.Parent->interval.second + curNode.g - curNode.Parent->g || curNodeIntervals[i].second < curNode.g)
                {
                    curNodeIntervals.erase(curNodeIntervals.begin() + i);
                    EAT.erase(EAT.begin() + i);
                    i--;
                }
        }
        return curNodeIntervals;
    }
}
bool ComboConstraints::hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision)
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
      A += VA*(startTimeB - startTimeA);
      startTimeA = startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
      B += VB*(startTimeA - startTimeB);
      startTimeB = startTimeA;
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
    if(dscr <= 0)
        return false;

    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    }
    else
        return false;
}

std::pair<double,double> ComboConstraints::countInterval(section sec, Node curNode)
{
    Point intersec, A(curNode.Parent->i, curNode.Parent->j), B(curNode.i, curNode.j), C(sec.i1, sec.j1), D(sec.i2, sec.j2);
    int pos = checkIntersection(A, B, C, D, intersec);
    int A1(A.j - B.j), A2(C.j - D.j), B1(A.i - B.i), B2(C.i - D.i);
    double lengthAB = curNode.g - curNode.Parent->g;
    double lengthCD = sec.g2 - sec.g1;
    if(A2 == 0 && B2 == 0)//if we collide with a section, that represents wait action (or goal)
    {
        double dist_to_AB = (B1*D.j - A1*D.i + A.j*B.i - A.i*B.j)/lengthAB;
        double gap = sqrt(1.0 - pow(dist_to_AB, 2));
        double offset = sqrt(pow(dist(B, C), 2) - pow(dist_to_AB, 2));
        return {sec.g1 + offset - gap, sec.g2 + offset + gap};
    }
    if(pos == CN_COINCIDENT || pos == CN_PARALLEL)
    {
        if(pos == CN_PARALLEL)
            if(abs(B1*sec.j2 - A1*sec.i2 + curNode.Parent->j*curNode.i - curNode.Parent->i*curNode.j) >= lengthAB)//if the distance between sections is not less than 1.0 (2r), collision is immpossible
                return {-1, -1};
        double BC = dist(B, C);
        if(A1*A2 >= 0 && B1*B2 >= 0)//if sections are co-directional
            return {sec.g1 + BC - 1, sec.g1 + BC + 1};
        if((A.i - C.i)*(A.i - D.i) <= 0 && (A.j - C.j)*(A.j - D.j) <= 0)//A inside CD
        {
            if((B.i - C.i)*(B.i - D.i) <= 0 && (B.j - C.j)*(B.j - D.j) <= 0)//B inside CD => AB is fully in CD
                return {sec.g1 + BC - 2, sec.g1 + BC + 2*lengthAB + 2};
            else
                return {sec.g1 + BC - 2, sec.g1 + BC + 2*(lengthAB - BC) + 2};
        }
        else//A outside of CD
        {
            if((B.i - C.i)*(B.i - D.i) <= 0 && (B.j - C.j)*(B.j - D.j) <= 0)//B inside CD
                return {sec.g1 + BC - 2, sec.g1 + BC + 2*(lengthCD - BC) + 2};
            else
                return {sec.g1 + BC - 2, sec.g1 + BC + 2*lengthCD + 2};
        }
    }
    else if(pos == CN_NONINTERSECTING)
    {
        double A_CD(minDist(A, C, D)), B_CD(minDist(B, C, D)), C_AB(minDist(C, A, B)), D_AB(minDist(D, A, B));
        if(min(min(A_CD, B_CD), min(C_AB, D_AB)) >= 1.0)
            return {-1,-1};

        intersec.i = ((C.i*D.j - C.j*D.i)*B1 - B2*(A.i*B.j - A.j*B.i))/((C.i - D.i)*(A.j - B.j) - (C.j - D.j)*(A.i - B.i));
        intersec.j = ((C.i*D.j - C.j*D.i)*A1 - A2*(A.i*B.j - A.j*B.i))/((C.i - D.i)*(A.j - B.j) - (C.j - D.j)*(A.i - B.i));
        int classAB = intersec.classify(A, B);
        int classCD = intersec.classify(C, D);
        double span = sqrt(2.0/((A1*A2 + B1*B2)/(sqrt(A1*A1 + B1*B1)*sqrt(A2*A2 + B2*B2)) + 1.0));
        std::pair<double, double> interval, interval2(-1,-1);
        if(classAB == 3 && classCD == 4)//intersection point is behind AB and beyond CD
        {
            double dist_A(sqrt(pow(A.i - intersec.i,2) + pow(A.j - intersec.j,2))),
                   dist_B(sqrt(pow(B.i - intersec.i,2) + pow(B.j - intersec.j,2))),
                   dist_C(sqrt(pow(C.i - intersec.i,2) + pow(C.j - intersec.j,2))),
                   dist_D(sqrt(pow(D.i - intersec.i,2) + pow(D.j - intersec.j,2))),
                   gap, offset;
            if(dist_A > dist_D)
            {
                gap = sqrt(1.0 - pow(A_CD, 2));
                offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(D_AB, 2));
                offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                interval = {sec.g2 + offset - gap, sec.g2 + offset + gap};
            }
            if(min(dist_B,dist_C)*2>span)
                return {sec.g1 + dist_C + dist_B - span, interval.second};
            else if(dist_B<dist_C)
            {
                gap = sqrt(1.0 - pow(B_CD, 2));
                offset = sqrt(pow(B.i - C.i,2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1 - pow(C_AB, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
        }
        else if(classAB == 4 && classCD == 3)
        {
            double dist_A(sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2))),
                   dist_B(sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2))),
                   dist_C(sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2))),
                   dist_D(sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2))),
                   gap, offset;
            if(dist_B > dist_C)
            {
                gap = sqrt(1.0 - pow(B_CD, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(C_AB, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            if(min(dist_A, dist_D)*2 > span)
                return {interval.first, sec.g1 - dist_C - dist_B + span};//agree
            else if(dist_A < dist_D)
            {
                gap = sqrt(1.0 - pow(A_CD, 2));
                offset = sqrt(pow(dist(A, C), 2) - pow(A_CD, 2)) + lengthAB;
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(D_AB, 2));
                offset = sqrt(pow(dist(B, D), 2) - pow(D_AB, 2));
                interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
            }
        }
        else if(classAB==3 && classCD==3)//intersection point is before both sections
        {
            double dist_A(sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2))),
                   dist_B(sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2))),
                   dist_C(sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2))),
                   dist_D(sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2))),
                   gap, offset;
            if(dist_A>dist_C)
            {
                gap = sqrt(1.0 - pow(A_CD, 2));
                offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(C_AB, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            if(min(dist_B, dist_D)*2 > span)
                return {sec.g1 - dist_C + dist_B - span, interval.second};
            else if(dist_B < dist_D)
            {
                gap = sqrt(1.0 - pow(B_CD, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(D_AB, 2));
                offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
            }
        }
        else if(classAB == 4 && classCD == 4)//intersection point is beyond both sections
        {
            double dist_A(sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2))),
                   dist_B(sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2))),
                   dist_C(sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2))),
                   dist_D(sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2))),
                   gap, offset;
            if(dist_B > dist_D)
            {
                gap = sqrt(1.0 - pow(B_CD, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(D_AB, 2));
                offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                interval = {sec.g2 + offset - gap, sec.g2 + offset + gap};
            }
            if(min(dist_A, dist_C)*2>span)
                return {sec.g2 + dist_D - dist_B - span, interval.second};
            else if(dist_A < dist_C)
            {
                gap = sqrt(1.0 - pow(A_CD, 2));
                offset = sqrt(pow(A.i - C.i, 2) +pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1 - pow(C_AB, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
        }
        else if(classAB == 4)//BEYOND (AFTER B)
        {
            double gap = sqrt(1.0 - pow(B_CD, 2));
            double offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
            interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};

            if((span - sqrt(2.0)) < CN_EPSILON)
            {
                double dist_A = sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2));
                double dist_C = sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2));
                if(min(dist_A, dist_C)*2> span)
                {
                    if(dist(B, intersec)*2 < span)
                        return {interval.first, sec.g1 + dist_C - dist(B, intersec) + span};
                    else
                        return interval;
                }
                else if(dist_A < dist_C)
                {
                    gap = sqrt(1.0 - pow(A_CD, 2));
                    offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1 - pow(C_AB, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
            }
            else
            {
                double dist_A = sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2));
                double dist_D = sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2));
                if(min(dist_A, dist_D)*2 > span)
                {
                    if(dist(B, intersec)*2 < span)
                        return {interval.first, sec.g1 + dist(C, intersec) - dist(B, intersec) + span};//not checked, mb +dist(B,intersec) is needed
                    else
                        return interval;
                }
                else if(dist_A < dist_D)
                {
                    gap = sqrt(1.0 - pow(A_CD, 2));
                    offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                    interval2 = {sec.g1 + offset - gap,sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(D_AB, 2));
                    offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                    interval2 = {sec.g2 + offset - gap,sec.g2 + offset + gap};
                }
            }

        }
        else if(classAB == 3)//BEHIND (BEFORE A)
        {
            double gap = sqrt(1.0 - pow(A_CD, 2));
            double offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
            interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            if((span - sqrt(2.0)) < CN_EPSILON)
            {
                double dist_B = sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2));
                double dist_D = sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2));
                if(min(dist_B, dist_D)*2 > span)
                {
                    if(dist(A, intersec)*2 < span)
                        return {sec.g1 + dist(C, intersec) + dist_B - span, interval.second};//changed
                    else
                        return interval;
                }
                else if(dist_B < dist_D)
                {
                    gap = sqrt(1.0 - pow(B_CD, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(D_AB, 2));
                    offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                    interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
                }
            }
            else
            {
                double dist_B = sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2));
                double dist_C = sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2));
                if(min(dist_B, dist_C)*2 > span)//checked
                {
                    if(dist(A, intersec)*2 < span)
                        return {sec.g1 + dist_C + dist_B - span, interval.second};
                    else
                        return interval;
                }
                else if(dist_B < dist_C)
                {
                    gap = sqrt(1.0 - pow(B_CD, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(C_AB, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
            }
        }
        else if(classCD == 4)//BEYOND (AFTER D)
        {
            double gap = sqrt(1.0 - pow(D_AB, 2));
            double offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
            interval = {sec.g2 + offset - gap, sec.g2 + offset + gap};

            if((span - sqrt(2.0)) < CN_EPSILON)
            {
                double dist_A = sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2));
                double dist_C = sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2));
                if(min(dist_A, dist_C)*2 > span)
                {
                    if(dist(D, intersec)*2 < span)
                        return {sec.g2 + dist(D, intersec) + dist(B, intersec) - span, interval.second};//agree
                    else
                        return interval;
                }
                else if(dist_A < dist_C)
                {
                    gap = sqrt(1.0 - pow(A_CD, 2));
                    offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(C_AB, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
            }
            else
            {
                double dist_B = sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2));
                double dist_C = sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2));
                if(min(dist_B, dist_C)*2 > span)//checked
                {
                    if(dist(D, intersec)*2 < span)
                        return {sec.g2 + dist(D, intersec) + dist_B - span, interval.second};
                    else
                        return interval;
                }
                else if(dist_B < dist_C)
                {
                    gap = sqrt(1.0 - pow(B_CD, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(C_AB, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
            }
        }
        else if(classCD == 3)//BEHIND (BEFORE C)
        {
            double gap = sqrt(1.0 - pow(C_AB, 2));
            double offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
            interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            if((span - sqrt(2.0)) < CN_EPSILON)//if sections are co-directional
            {
                double dist_B = sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2));
                double dist_D = sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2));
                if(min(dist_B, dist_D)*2 > span)//checked
                {
                    if(dist(C, intersec)*2 < span)
                        return {interval.first, sec.g1 - dist(C, intersec) + dist_B + span};
                    else
                        return interval;
                }
                else if(dist_B < dist_D)
                {
                    gap = sqrt(1.0 - pow(B_CD, 2));
                    offset = sqrt(pow(B.i - C.i,2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(D_AB, 2));
                    offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                    interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
                }
            }
            else//checked
            {
                double dist_A = sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2));
                double dist_D = sqrt(pow(D.i - intersec.i,2) + pow(D.j - intersec.j, 2));
                if(min(dist_A, dist_D)*2 > span)
                {
                    if(dist(C, intersec)*2 < span)
                        return {interval.first, sec.g1 - dist(C, intersec) + dist(B, intersec) + span};
                    else
                        return interval;
                }
                else if(dist_A<dist_D)
                {
                    gap = sqrt(1.0 - pow(A_CD, 2));
                    offset = sqrt(pow(A.i - C.i,2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(D_AB, 2));
                    offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                    interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
                }
            }
        }
        return {min(interval.first, interval2.first), max(interval.second, interval2.second)};
    }
    else//have intersection point
    {

        double dist_A(sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2))),
                dist_B(sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2))),
                dist_C(sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2))),
                dist_D(sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2))),
                span = sqrt(2.0/((A1*A2 + B1*B2)/(sqrt(A1*A1 + B1*B1)*sqrt(A2*A2 + B2*B2)) + 1.0)),
                dist;
        std::pair<double,double> interval(sec.g1 + dist_C + dist_B - span, sec.g1 + dist_C + dist_B + span);

        if(min(dist_A, dist_D)*2 < span)
        {
            if(dist_A < dist_D)
            {
                dist = ((C.i - D.i)*A.j + (D.j - C.j)*A.i + (C.j*D.i - D.j*C.i))/lengthCD;
                interval.second = sec.g1 + sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(dist, 2)) + lengthAB + sqrt(1.0 - pow(dist, 2));
            }
            else
            {
                dist = ((A.i - B.i)*D.j + (B.j - A.j)*D.i + (A.j*B.i - A.i*B.j))/lengthAB;
                interval.second = sec.g2 + sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(dist, 2)) + sqrt(1.0 - pow(dist, 2));
            }
        }
        if(min(dist_B, dist_C)*2 < span)
        {
            if(dist_B < dist_C)
                dist = ((C.i - D.i)*B.j + (D.j - C.j)*B.i + (C.j*D.i - D.j*C.i))/lengthCD;
            else
                dist = ((A.i - B.i)*C.j + (B.j - A.j)*C.i + (A.j*B.i - A.i*B.j))/lengthAB;
            interval.first = sec.g1 + sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(dist, 2)) - sqrt(1.0 - pow(dist, 2));
        }
        return interval;
    }
}
