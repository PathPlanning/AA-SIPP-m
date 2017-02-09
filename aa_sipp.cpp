#include "aa_sipp.h"

bool sort_function(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}


AA_SIPP::AA_SIPP(double weight, bool breakingties)
{
    this->weight = weight;
    this->breakingties = breakingties;
    closeSize = 0;
    openSize = 0;
    gap = 2;//equivalent of 4r
}

AA_SIPP::~AA_SIPP()
{
}

bool AA_SIPP::stopCriterion()
{
    if(openSize == 0)
    {
        std::cout << "OPEN list is empty! " << std::endl;
        return true;
    }
    return false;
}

void AA_SIPP::findSuccessors(const Node curNode, const cMap &Map, std::list<Node> &succs, int numOfCurAgent)
{
    Node newNode;
    std::vector<double> EAT;
    std::vector<std::pair<double, double>> intervals;
    double h_value;
    auto parent = &(close.find(curNode.i*Map.width + curNode.j)->second);
    for(int i = -1; i <= +1; i++)
    {
        for(int j = -1; j <= +1; j++)
        {
            if((i != 0 || j != 0) && Map.CellOnGrid(curNode.i + i, curNode.j + j) && Map.CellIsTraversable(curNode.i + i, curNode.j + j))
            {
                if((i*j != 0) && (Map.CellIsObstacle(curNode.i, curNode.j + j) || Map.CellIsObstacle(curNode.i + i, curNode.j)))
                    continue;
                newNode.i = curNode.i + i;
                newNode.j = curNode.j + j;
                newNode.g = curNode.g + MoveCost(curNode.i, curNode.j, curNode.i + i, curNode.j + j);
                newNode.Parent = parent;
                EAT.clear();
                h_value = weight*calculateDistanceFromCellToCell(newNode.i, newNode.j, Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]);
                intervals = findIntervals(newNode, EAT);
                for(int k = 0; k < intervals.size(); k++)
                {
                    newNode.interval = intervals[k];
                    newNode.g = EAT[k];
                    newNode.F = newNode.g + h_value;
                    succs.push_front(newNode);
                }
                newNode = resetParent(newNode, curNode, Map);
                if(newNode.Parent->i != parent->i || newNode.Parent->j != parent->j)
                {
                    EAT.clear();
                    intervals = findIntervals(newNode, EAT);
                    for(int k = 0; k < intervals.size(); k++)
                    {
                        newNode.interval = intervals[k];
                        newNode.g = EAT[k];
                        newNode.F = newNode.g + h_value;
                        succs.push_front(newNode);
                    }
                }
            }
        }
    }
}

double AA_SIPP::MoveCost(int start_i, int start_j, int fin_i, int fin_j)
{
    if((start_i - fin_i)*(start_j - fin_j) != 0)
        return sqrt(2);
    return 1;
}

bool AA_SIPP::lineOfSight(int i1, int j1, int i2, int j2, const cMap &map)
{
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    int sep_value = delta_i*delta_i + delta_j*delta_j;
    if(delta_i == 0)
    {
        for(; j != j2; j += step_j)
            if(map.CellIsObstacle(i, j))
                return false;
    }
    else if(delta_j == 0)
    {
        for(; i != i2; i += step_i)
            if(map.CellIsObstacle(i, j))
                return false;
    }
    else if(delta_i > delta_j)
    {
        for(; i != i2; i += step_i)
        {
            if(map.CellIsObstacle(i, j))
                return false;
            if(map.CellIsObstacle(i, j + step_j))
                return false;
            error += delta_j;
            if(error >= delta_i)
            {
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    if(map.CellIsObstacle(i + step_i, j))
                        return false;
                if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                    if(map.CellIsObstacle(i, j + 2*step_j))
                        return false;
                j += step_j;
                error -= delta_i;
            }
        }
        if(map.CellIsObstacle(i, j))
            return false;
    }
    else
    {
        for(; j != j2; j += step_j)
        {
            if(map.CellIsObstacle(i, j))
                return false;
            if(map.CellIsObstacle(i + step_i, j))
                return false;
            error += delta_i;
            if(error >= delta_j)
            {
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < (delta_i*delta_i + delta_j*delta_j))
                    if(map.CellIsObstacle(i, j + step_j))
                        return false;
                if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i))<(delta_i*delta_i+delta_j*delta_j))
                    if(map.CellIsObstacle(i + 2*step_i, j))
                        return false;
                i += step_i;
                error -= delta_j;
            }
        }
        if(map.CellIsObstacle(i, j))
            return false;
    }
    return true;
}

double AA_SIPP::calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j)
{
    return sqrt(double((start_i - fin_i)*(start_i - fin_i) + (start_j - fin_j)*(start_j - fin_j)));
}

Node AA_SIPP::findMin(int size)
{
    Node min;
    min.F = std::numeric_limits<float>::max();
    for(int i = 0; i < size; i++)
    {
        if(open[i].size() != 0 && open[i].begin()->F <= min.F)
        {
            if (open[i].begin()->F == min.F)
            {
                if (open[i].begin()->g >= min.g)
                    min=*open[i].begin();
            }
            else
                min=*open[i].begin();
        }
    }
    return min;
}

void AA_SIPP::addOpen(Node &newNode)
{
    std::list<Node>::iterator iter, pos;
    bool posFound = false;
    pos = open[newNode.i].end();
    if (open[newNode.i].size() == 0)
    {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }
    for(iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
    {
        if ((iter->F >= newNode.F) && (!posFound))
        {
            if (iter->F == newNode.F)
            {

                if ((newNode.g > iter->g && breakingties == CN_BT_G_MAX) || (newNode.g < iter->g && breakingties == CN_BT_G_MIN))
                {
                    pos = iter;
                    posFound = true;
                }
            }
            else
            {
                pos = iter;
                posFound = true;
            }
        }

        if (iter->i == newNode.i && iter->j == newNode.j && iter->interval.first == newNode.interval.first)
        {
            if(iter->g - newNode.g < CN_EPSILON)//CN_EPSILON is needed to prevent mistakes with comparison of double-type values
                return;
            if(pos == iter)
            {
                iter->F = newNode.F;
                iter->g = newNode.g;
                iter->interval = newNode.interval;
                iter->Parent = newNode.Parent;
                return;
            }
            open[newNode.i].erase(iter);
            openSize--;
            break;
        }
    }
    openSize++;
    open[newNode.i].insert(pos, newNode);
}

SearchResult AA_SIPP::startSearch(cLogger *Log, cMap &Map)
{
#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    sresult.pathInfo.resize(Map.agents);
    sresult.agents = Map.agents;
    sresult.agentsSolved = 0;
    ctable.resize(Map.height);
    safe_intervals.resize(Map.height);
    for(int i=0; i<Map.height; i++)
    {
        ctable[i].resize(Map.width);
        safe_intervals[i].resize(Map.width);
        for(int j=0; j<Map.width; j++)
        {
            ctable[i][j].resize(0);
            safe_intervals[i][j].resize(0);
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
    for(int i = 0; i < Map.agents; i++)
    {
        Map.addConstraint(Map.start_i[i], Map.start_j[i]);
        Map.addConstraint(Map.goal_i[i], Map.goal_j[i]);
    }
    for(int numOfCurAgent = 0; numOfCurAgent < Map.agents; numOfCurAgent++)
    {
        Map.removeConstraint(Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent]);
        Map.removeConstraint(Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]);
        if(findPath(numOfCurAgent, Map))
            addConstraints(numOfCurAgent);
        close.clear();
        for(int i = 0; i< Map.height; i++)
            open[i].clear();
        delete [] open;
    }
#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#else
    QueryPerformanceCounter(&end);
    sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
    std::vector<conflict> confs = CheckConflicts();
    for(int i = 0; i < confs.size(); i++)
        std::cout<<confs[i].i<<" "<<confs[i].j<<" "<<confs[i].g<<" "<<confs[i].agent1<<" "<<confs[i].agent2<<"\n";

    return sresult;
}


Node AA_SIPP::resetParent(Node current, Node Parent, const cMap &Map)
{
    if(Parent.Parent == NULL || (Parent.Parent->i == current.i && Parent.Parent->j == current.j))
        return current;

    double g_value = calculateDistanceFromCellToCell(Parent.Parent->i, Parent.Parent->j, current.i, current.j);
    //if(g_value > 15)
    //    return current;
    if(lineOfSight(Parent.Parent->i, Parent.Parent->j, current.i, current.j, Map))
    {
        current.g = Parent.Parent->g + g_value;
        current.Parent = Parent.Parent;
    }
    return current;
}


std::vector<constraint> AA_SIPP::findConflictCells(Node cur)
{
    std::vector<constraint> cells(0);
    int i1 = cur.i, j1 = cur.j, i2 = cur.Parent->i, j2 = cur.Parent->j;
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    int sep_value = delta_i*delta_i + delta_j*delta_j;
    constraint add;
    if((delta_i + delta_j) == 0)//this situation is possible after modification of hppath and is needed for addConstraints function
    {
        add.i=i;
        add.j=j;
        cells.push_back(add);
    }
    else if(delta_i == 0)
    {
        add.i = i1;
        for(; j != j2; j += step_j)
        {
            add.j = j;
            cells.push_back(add);
        }
        add.j=j2;
        cells.push_back(add);
    }
    else if(delta_j == 0)
    {
        add.j = j1;
        for(; i != i2; i += step_i)
        {
            add.i = i;
            cells.push_back(add);
        }
        add.i=i2;
        cells.push_back(add);
    }
    else if(delta_i > delta_j)
    {
        for(; i != i2; i += step_i)
        {
            add.i = i;
            add.j = j;
            cells.push_back(add);
            add.j += step_j;
            cells.push_back(add);
            add.j -= step_j;
            error += delta_j;
            if(error >= delta_i)
            {
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                {
                    add.i += step_i;
                    cells.push_back(add);
                    add.i -= step_i;
                }
                if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                {
                    add.j += 2*step_j;
                    cells.push_back(add);
                    add.j -= 2*step_j;
                }
                j += step_j;
                error -= delta_i;
            }
        }
        add.i = i;
        add.j = j;
        cells.push_back(add);
    }
    else
    {
        for(; j != j2; j += step_j)
        {
            add.i = i;
            add.j = j;
            cells.push_back(add);
            add.i += step_i;
            cells.push_back(add);
            add.i -= step_i;
            error += delta_i;
            if(error >= delta_j)
            {
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                {
                    add.j += step_j;
                    cells.push_back(add);
                    add.j -= step_j;
                }
                if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < sep_value)
                {
                    add.i += 2*step_i;
                    cells.push_back(add);
                    add.i -= 2*step_i;
                }
                i += step_i;
                error -= delta_j;
            }
        }
        add.i = i;
        add.j = j;
        cells.push_back(add);
    }
    return cells;
}

void AA_SIPP::addConstraints(int curAgent)
{
    Node cur;
    std::vector<constraint> cells;
    for(int a = 1; a < sresult.pathInfo[curAgent].sections.size(); a++)
    {
        cur = sresult.pathInfo[curAgent].sections[a];
        cells = findConflictCells(cur);
        int x1 = cur.i, y1 = cur.j, x0 = cur.Parent->i, y0 = cur.Parent->j;
        if(x1 != x0 || y1 != y0)
            cur.Parent->g = cur.g - calculateDistanceFromCellToCell(x0, y0, x1, y1);
        constraint add;
        add.agent = curAgent;
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
                if(ctable[add.i][add.j].empty() || ctable[add.i][add.j].back().g != add.g)
                    ctable[add.i][add.j].push_back(add);
            }
            if(ctable[add.i][add.j].back().g < cur.g)
            {
                add.g = cur.g;
                ctable[add.i][add.j].push_back(add);
            }
            continue;
        }
        else
        {
            for(int i = 0; i < cells.size(); i++)
            {
                add = cells[i];
                add.agent = curAgent;
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
                con.g = cur.Parent->g + calculateDistanceFromCellToCell(cur.Parent->i, cur.Parent->j, con.i, con.j);
                if(add.i == sresult.pathInfo[curAgent].sections.back().i && add.j == sresult.pathInfo[curAgent].sections.back().j)
                    con.goal = true;
                else
                    con.goal = false;
                con.agent = curAgent;
                ctable[add.i][add.j].push_back(con);
            }
            for(int i = 0; i < cells.size(); i++)
            {
                add = cells[i];
                add.agent = curAgent;
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
                        if(safe_intervals[add.i][add.j][j].second < interval.second)
                        {
                            safe_intervals[add.i][add.j].erase(safe_intervals[add.i][add.j].begin() + j);
                            break;
                        }
                        else if(safe_intervals[add.i][add.j][j].first == interval.first)
                        {
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
                    else if(safe_intervals[add.i][add.j][j].first > interval.first && safe_intervals[add.i][add.j][j].first < interval.second)
                    {
                        if(safe_intervals[add.i][add.j][j].second < interval.second)
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
}

bool AA_SIPP::findPath(int numOfCurAgent, const cMap &Map)
{

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    open = new std::list<Node>[Map.height];
    ResultPathInfo resultPath;
    openSize = 0;
    closeSize = 0;

    Node curNode(Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent], 0, 0);
    curNode.F = weight * calculateDistanceFromCellToCell(curNode.i, curNode.j, Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]);
    curNode.interval = {0, CN_INFINITY};
    bool pathFound = false;
    open[curNode.i].push_back(curNode);
    openSize++;
    while(!stopCriterion())
    {
        curNode = findMin(Map.height);
        open[curNode.i].pop_front();
        openSize--;
        close.insert({curNode.i * Map.width + curNode.j, curNode});
        closeSize++;
        if(curNode.i == Map.goal_i[numOfCurAgent] && curNode.j == Map.goal_j[numOfCurAgent] && curNode.interval.second == CN_INFINITY)
        {
            pathFound = true;
            break;
        }
        std::list<Node> successors;
        successors.clear();
        findSuccessors(curNode, Map, successors, numOfCurAgent);
        auto it = successors.begin();
        while(it != successors.end())
        {
            bool has = false;
            auto range = close.equal_range(it->i * Map.width + it->j);
            for(auto i = range.first; i != range.second; i++)
                if(i->second.interval.first == it->interval.first && i->second.interval.second == it->interval.second)
                {
                    has = true;
                    break;
                }
            if(!has)
                addOpen(*it);
            it++;
        }
    }

    if (pathFound)
    {
        makePrimaryPath(curNode);
        for(int i = 1; i < hppath.size(); i++)
            if((hppath[i].g - (hppath[i - 1].g + calculateDistanceFromCellToCell(hppath[i].i, hppath[i].j, hppath[i - 1].i, hppath[i - 1].j))) > 1e-4)
            {
                Node add = hppath[i - 1];
                add.Parent = hppath[i].Parent;
                close.insert({add.i*Map.width + add.j, add});
                hppath[i].Parent = &(close.find(add.i*Map.width + add.j)->second);
                add.g = hppath[i].g - calculateDistanceFromCellToCell(hppath[i].i, hppath[i].j, hppath[i - 1].i,hppath[i - 1].j);
                hppath.emplace(hppath.begin() + i, add);
                i++;
            }
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        resultPath.sections = hppath;
        makeSecondaryPath(curNode);
        resultPath.nodescreated = openSize + closeSize;
        resultPath.pathfound = true;
        resultPath.path = lppath;
        resultPath.numberofsteps = closeSize;
        resultPath.pathlength = curNode.g;
        sresult.pathfound = true;
        sresult.pathlength += curNode.g;
        sresult.nodescreated += openSize + closeSize;
        sresult.numberofsteps += closeSize;
        sresult.pathInfo[numOfCurAgent] = resultPath;
        sresult.agentsSolved++;
    }
    else
    {
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        std::cout<<numOfCurAgent<<" PATH NOT FOUND!\n";
        sresult.pathfound = false;
        sresult.nodescreated += closeSize;
        sresult.numberofsteps += closeSize;
        resultPath.nodescreated = closeSize;
        resultPath.pathfound = false;
        resultPath.path.clear();
        resultPath.sections.clear();
        resultPath.pathlength = 0;
        resultPath.numberofsteps = closeSize;
        sresult.pathInfo[numOfCurAgent] = resultPath;
    }
    return resultPath.pathfound;
}

std::vector<std::pair<double,double>> AA_SIPP::findIntervals(Node curNode, std::vector<double> &EAT)
{
    std::vector<std::pair<double,double>> badIntervals(0), curNodeIntervals(0);
    double dist = curNode.g - curNode.Parent->g;

    for(int i = 0; i < safe_intervals[curNode.i][curNode.j].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][i].second > curNode.Parent->g + dist && safe_intervals[curNode.i][curNode.j][i].first <= curNode.Parent->interval.second + dist)
        {
            curNodeIntervals.push_back(safe_intervals[curNode.i][curNode.j][i]);
            if(curNodeIntervals.back().first < curNode.Parent->g + dist)
                EAT.push_back(curNode.Parent->g + dist);
            else
                EAT.push_back(curNodeIntervals.back().first);
        }
    std::vector<constraint> cells = findConflictCells(curNode);

    double ab = curNode.g - curNode.Parent->g;
    double da, db, ha, vi, vj, wi, wj, c1, c2;
    std::pair<double,double> add;
    constraint con;
    for(int i = 0; i < cells.size(); i++)
    {
        for(int j = 0; j < ctable[cells[i].i][cells[i].j].size(); j++)
        {

            con = ctable[cells[i].i][cells[i].j][j];
            if(con.g + gap < curNode.Parent->g && !con.goal)
                continue;
            da = ((curNode.i - con.i)*(curNode.i - con.i) + (curNode.j - con.j)*(curNode.j - con.j));
            db = ((curNode.Parent->i - con.i)*(curNode.Parent->i - con.i) +(curNode.Parent->j - con.j)*(curNode.Parent->j - con.j));
            vi = curNode.Parent->i - curNode.i;
            vj = curNode.Parent->j - curNode.j;
            wi = con.i - curNode.i;
            wj = con.j - curNode.j;
            c1 = vj*wj + vi*wi;
            c2 = vj*vj + vi*vi;
            if(c1 <= 0 || c2 <= c1)//if constraint is outside of the segment
            {
                if(da < db)
                    dist = da;
                else
                    dist = db;
                if(dist < 1)
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
            else if(con.i == curNode.i && con.j == curNode.j)
            {
                add = {con.g - gap, con.g + gap};
                if(con.goal == true)
                    add.second = CN_INFINITY;
                badIntervals.push_back(add);
                continue;
            }
            else if(con.i==curNode.Parent->i && con.j==curNode.Parent->j)
            {
                add = {con.g - gap + ab, con.g + gap + ab};
                if(con.goal == true)
                    add.second = CN_INFINITY;
                badIntervals.push_back(add);
                continue;
            }
            else
            {
                dist = fabs(((curNode.Parent->i - curNode.i)*con.j + (curNode.j - curNode.Parent->j)*con.i
                             +(curNode.Parent->j*curNode.i - curNode.Parent->i*curNode.j)))
                        /sqrt(pow(curNode.Parent->i - curNode.i, 2) + pow(curNode.Parent->j - curNode.j, 2));
                if(dist < 1)
                {
                    ha = sqrt(da - dist*dist);
                    add = {con.g - gap + ha, con.g + gap + ha};
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
            if(cur.first == next.first)
            {
                if(next.second >= cur.second)
                    badIntervals[i].second = next.second;
                badIntervals.erase(badIntervals.begin() + i + 1);
                i--;
            }
            else if((next.first - cur.first)*(next.second - cur.first) <= 0)
            {
                if((cur.first - next.second)*(cur.second - next.second) <= 0)
                    badIntervals[i].first = next.first;
                badIntervals.erase(badIntervals.begin() + i + 1);
                i--;
            }
            else if((cur.first - next.first)*(cur.second - next.first) <= 0)
            {
                if((cur.second - next.first)*(cur.second - next.second) <= 0)
                    badIntervals[i].second = next.second;
                badIntervals.erase(badIntervals.begin() + i + 1);
                i--;
            }
        }
    }

    //searching reachebale intervals and theirs EAT
    if(badIntervals.size() > 0)
    {
        std::pair<double,double> parentInterval = curNode.Parent->interval;
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
        dist = curNode.g - curNode.Parent->g;
        for(int i = 0; i < curNodeIntervals.size(); i++)
            if(EAT[i] > parentInterval.second + dist || curNodeIntervals[i].second < curNode.g)
            {
                curNodeIntervals.erase(curNodeIntervals.begin() + i);
                EAT.erase(EAT.begin() + i);
                i--;
            }
    }
    return curNodeIntervals;
}

std::vector<conflict> AA_SIPP::CheckConflicts()
{
    std::vector<conflict> conflicts(0);
    conflict conf;
    Node cur, check;
    std::vector<std::vector<conflict>> positions;
    positions.resize(sresult.agents);
    for(int i = 0; i < sresult.agents; i++)
    {
        if(!sresult.pathInfo[i].pathfound)
            continue;
        positions[i].resize(0);
        int k = 0;
        double part = 1;
        for(int j = 1; j<sresult.pathInfo[i].sections.size(); j++)
        {
            cur = sresult.pathInfo[i].sections[j];
            check = sresult.pathInfo[i].sections[j-1];
            int di = cur.i - check.i;
            int dj = cur.j - check.j;
            double dist = (cur.g - check.g)*10;
            int steps = (cur.g - check.g)*10;
            if(dist - steps + part >= 1)
            {
                steps++;
                part = dist - steps;
            }
            else
                part += dist - steps;
            double stepi = double(di)/dist;
            double stepj = double(dj)/dist;
            double curg = double(k)*0.1;
            double curi = check.i + (curg - check.g)*di/(cur.g - check.g);
            double curj = check.j + (curg - check.g)*dj/(cur.g - check.g);
            conf.i = curi;
            conf.j = curj;
            conf.g = curg;
            positions[i].push_back(conf);
            k++;
            while(curg <= cur.g)
            {
                if(curg + 0.1 > cur.g)
                    break;
                curi += stepi;
                curj += stepj;
                curg += 0.1;
                conf.i = curi;
                conf.j = curj;
                conf.g = curg;
                positions[i].push_back(conf);
                k++;
            }
        }
        if(double(k - 1)*0.1 < sresult.pathInfo[i].sections.back().g)
        {
            conf.i = sresult.pathInfo[i].sections.back().i;
            conf.j = sresult.pathInfo[i].sections.back().j;
            conf.g = sresult.pathInfo[i].sections.back().g;
            positions[i].push_back(conf);
        }
    }
    int max = 0;
    for(int i = 0; i < positions.size(); i++)
        if(positions[i].size() > max)
            max = positions[i].size();
    for(int i = 0; i < sresult.agents; i++)
    {
        for(int k = 0; k < max; k++)
        {
            for(int j = i + 1; j < sresult.agents; j++)
            {
                if(!sresult.pathInfo[j].pathfound || !sresult.pathInfo[i].pathfound)
                    continue;
                conflict a, b;
                if(positions[i].size() > k)
                    a = positions[i][k];
                else
                    a = positions[i].back();
                if(positions[j].size() > k)
                    b = positions[j][k];
                else
                    b = positions[j].back();
                if(((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j)) + CN_EPSILON < 1.0)
                {
                    conf.i = b.i;
                    conf.j = b.j;
                    conf.agent1 = i;
                    conf.agent2 = j;
                    conf.g = b.g;
                    conflicts.push_back(conf);
                }
            }
        }
    }
    return conflicts;
}

void AA_SIPP::makePrimaryPath(Node curNode)
{
    hppath.clear();
    hppath.shrink_to_fit();
    std::list<Node> path;
    path.push_front(curNode);
    if(curNode.Parent != NULL)
    {
        curNode = *curNode.Parent;
        if(curNode.Parent != NULL)
        {
            do
            {
                path.push_front(curNode);
                curNode = *curNode.Parent;
            }
            while(curNode.Parent != NULL);
        }
        path.push_front(curNode);
    }
    for(auto it = path.begin(); it != path.end(); it++)
        hppath.push_back(*it);
    return;
}

void AA_SIPP::makeSecondaryPath(Node curNode)
{
    if(curNode.Parent != NULL)
    {
        std::vector<Node> lineSegment;
        do
        {
            calculateLineSegment(lineSegment, *curNode.Parent, curNode);
            lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
            curNode = *curNode.Parent;
        }
        while(curNode.Parent != NULL);
        lppath.push_front(*lineSegment.begin());
    }
    else
        lppath.push_front(curNode);
}

void AA_SIPP::calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal)
{
    int i1 = start.i;
    int i2 = goal.i;
    int j1 = start.j;
    int j2 = goal.j;

    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    if (delta_i > delta_j)
    {
        for (; i != i2; i += step_i)
        {
            line.push_back(Node(i,j));
            error += delta_j;
            if ((error << 1) > delta_i)
            {
                j += step_j;
                error -= delta_i;
            }
        }
    }
    else
    {
        for (; j != j2; j += step_j)
        {
            line.push_back(Node(i,j));
            error += delta_i;
            if ((error << 1) > delta_j)
            {
                i += step_i;
                error -= delta_j;
            }
        }
    }
    return;
}
