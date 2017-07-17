#include "sipp.h"

SIPP::~SIPP()
{
}

SIPP::SIPP(double weight, int metrictype, bool breakingties)
{
    this->weight = weight;
    this->breakingties = breakingties;
    this->metrictype = metrictype;
    closeSize = 0;
    openSize = 0;
}

bool SIPP::stopCriterion()
{
    if(openSize == 0)
    {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }
    return false;
}

double SIPP::countHValue(int i, int j, int goal_i, int goal_j)
{
    if(metrictype == CN_MT_DIAGONAL)
        return std::min(abs(i - goal_i),abs(j - goal_j))*sqrt(2)+abs(abs(i - goal_i) - abs(j - goal_j));
    else if(metrictype == CN_MT_EUCLID)
        return sqrt((i - goal_i)*(i - goal_i) + (j - goal_j)*(j - goal_j));
    else if(metrictype == CN_MT_MANHATTAN)
        return abs(i - goal_i) + abs(j - goal_j);
}

void SIPP::findSuccessors(const Node curNode, const cMap &Map, std::list<Node> &succs, int numOfCurAgent)
{
    Node newNode;
    std::vector<std::pair<int,int>> intervals(0);
    std::pair<int,int> interval(-1, -1);

    for(int i = -1; i <= +1; i++)
    {
        for(int j = -1; j <= +1; j++)
        {
            if((i*j) == 0 && (i+j) != 0 && Map.CellOnGrid(curNode.i + i, curNode.j + j) && (Map.CellIsTraversable(curNode.i + i, curNode.j + j)))
            {
                newNode.i = curNode.i + i;
                newNode.j = curNode.j + j;
                newNode.g = curNode.g + 1;
                int direction = CN_UP_DIR;
                if(j == -1)
                    direction = CN_LEFT_DIR;
                else if(j == 1)
                    direction = CN_RIGHT_DIR;
                else if(i == 1)
                    direction = CN_DOWN_DIR;
                double h_value = weight*countHValue(newNode.i, newNode.j, Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]);
                if(ctable[newNode.i][newNode.j].size() != 0)
                {
                    intervals.clear();
                    if(ctable[newNode.i][newNode.j][0].g - 1 >= newNode.g)
                    {
                        interval = {newNode.g,ctable[newNode.i][newNode.j][0].g - 1};
                        if(ctable[newNode.i][newNode.j][0].g - 1 <= newNode.g)
                            interval.first++;
                        if(direction != ctable[newNode.i][newNode.j][0].s_dir)
                            interval.second--;
                        if(interval.second >= interval.first)
                            intervals.push_back(interval);
                    }
                    for(int k = 0; k < ctable[newNode.i][newNode.j].size(); k++)
                    {
                        if(ctable[newNode.i][newNode.j][k].s_dir == CN_GOAL_DIR)
                            break;
                        if(k < ctable[newNode.i][newNode.j].size() - 1)
                        {
                            if(ctable[newNode.i][newNode.j][k].g + 1 <= ctable[newNode.i][newNode.j][k + 1].g - 1)
                            {
                                interval = {ctable[newNode.i][newNode.j][k].g + 1, ctable[newNode.i][newNode.j][k + 1].g - 1};
                                if(direction != ctable[newNode.i][newNode.j][k].s_dir)
                                    interval.first++;
                                if(direction != ctable[newNode.i][newNode.j][k + 1].p_dir || direction != ctable[newNode.i][newNode.j][k + 1].s_dir)
                                    interval.second--;
                                if(interval.second >= interval.first)
                                    intervals.push_back(interval);
                            }
                        }
                        else
                        {
                            intervals.push_back({ctable[newNode.i][newNode.j][k].g + 1, CN_INFINITY});
                            if(direction != ctable[newNode.i][newNode.j][k].s_dir)
                                intervals.back().first++;
                        }
                    }
                    for(int k = 0; k < intervals.size(); k++)
                    {
                        if(intervals[k].second < curNode.interval.first + 1)
                            continue;
                        if(intervals[k].first >= curNode.interval.second + 1)
                            continue;
                        newNode.interval = intervals[k];
                        if(newNode.interval.first > newNode.g)
                            newNode.g = newNode.interval.first;
                        if(newNode.interval.second < newNode.g)
                            continue;
                        newNode.F = newNode.g + h_value;
                        succs.push_back(newNode);
                    }
                }
                else
                {
                    newNode.interval = {newNode.g, CN_INFINITY};
                    newNode.F = newNode.g + h_value;
                    if(newNode.interval.first >= curNode.interval.second + 1)
                        continue;
                    succs.push_back(newNode);
                }
            }
        }
    }
}

Node SIPP::findMin(int size)
{
    Node min;
    min.F = -1;
    for(int i = 0; i < size; i++)
    {
        if(open[i].size() != 0)
            if(open[i].begin()->F <= min.F || min.F == -1)
            {
                if (open[i].begin()->F == min.F)
                {
                    if (open[i].begin()->g >= min.g)
                        min = *open[i].begin();
                }
                else
                    min = *open[i].begin();
            }
    }
    return min;
}

void SIPP::addOpen(Node &newNode)
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
        if (iter->i == newNode.i && iter->j == newNode.j && iter->interval.first == newNode.interval.first && iter->interval.second == newNode.interval.second)
        {
            if(newNode.F >= iter->F)
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


SearchResult SIPP::startSearch(cLogger *Log, cMap &Map)
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
    for(int i = 0; i < Map.height; i++)
    {
        ctable[i].resize(Map.width);
        for(int j = 0; j < Map.width; j++)
            ctable[i][j].resize(0);
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
            addConstraints();
        close.clear();
        for(int i = 0; i < Map.height; i++)
            open[i].clear();
        delete [] open;
    }

#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.time = static_cast<double>(end.QuadPart - begin.QuadPart)/freq.QuadPart;
#endif
    std::vector<conflict> conflicts = CheckConflicts();
    for(int i = 0; i < conflicts.size(); i++)
        std::cout<< i << " " << conflicts[i].agent1 << " " << conflicts[i].agent2 << " " << conflicts[i].g << "\n";
    return sresult;
}

void SIPP::makePrimaryPath(Node curNode)
{
    hppath.clear();
    std::list<Node> path;
    if(curNode.Parent != NULL)
        do
    {
        path.push_front(curNode);
        curNode = *curNode.Parent;
    }
    while(curNode.Parent != NULL);
    path.push_front(curNode);
    for(auto it = path.begin(); it != path.end(); it++)
        hppath.push_back(*it);
}

void SIPP::makeSecondaryPath(Node curNode)
{
    if(curNode.Parent != NULL)
        do
    {
        lppath.push_front(curNode);
        curNode = *curNode.Parent;
    }
    while(curNode.Parent != NULL);
    lppath.push_front(curNode);
}

std::vector<conflict> SIPP::CheckConflicts()
{
    std::vector<conflict> conflicts(0);
    conflict conf;
    Node cur, check, curnext, checknext;
    for(int i = 0; i < sresult.agents; i++)
        for(int j = i + 1; j < sresult.agents; j++)
            for(int k = 0; k < sresult.pathInfo[i].sections.size(); k++)
                for(int l = 0; l < sresult.pathInfo[j].sections.size(); l++)
                {

                    cur = sresult.pathInfo[i].sections[k];
                    check = sresult.pathInfo[j].sections[l];
                    if(cur.i == check.i && cur.j == check.j && abs(cur.g-check.g)<=1)
                    {
                        conf.agent1 = i;
                        conf.agent2 = j;
                        conf.sec1 = k;
                        conf.sec2 = l;
                        conf.i = cur.i;
                        conf.j = cur.j;
                        conf.g = cur.g;
                        conflicts.push_back(conf);
                    }
                    if(k + 1 < sresult.pathInfo[i].sections.size() && l + 1<sresult.pathInfo[j].sections.size())
                    {
                        curnext = sresult.pathInfo[i].sections[k+1];
                        checknext = sresult.pathInfo[j].sections[l+1];
                        if(cur.i == checknext.i && cur.j == checknext.j && check.i == curnext.i && check.j == curnext.j && cur.g == check.g)
                        {
                            conf.agent1 = i;
                            conf.agent2 = j;
                            conf.sec1 = k;
                            conf.sec2 = l;
                            conf.i = cur.i;
                            conf.j = cur.j;
                            conf.g = cur.g;
                            conflicts.push_back(conf);
                        }
                        if(((cur.i == checknext.i && cur.j == checknext.j && cur.g==checknext.g) || (curnext.i == check.i && curnext.j == check.j && curnext.g == check.g)) && ((cur.i-curnext.i)!=(check.i-checknext.i) || (cur.j-curnext.j)!=(check.j-checknext.j)))
                        {
                            conf.agent1 = i;
                            conf.agent2 = j;
                            conf.sec1 = k;
                            conf.sec2 = l;
                            conf.i = cur.i;
                            conf.j = cur.j;
                            conf.g = cur.g;
                            conflicts.push_back(conf);
                        }
                    }
                }
    return conflicts;
}

void SIPP::addConstraints()
{
    Node cur;
    movement add;
    for(int i = 0; i < sresult.pathInfo.back().sections.size() - 1; i++)
    {
        cur = sresult.pathInfo.back().sections[i];
        add.g = cur.g;
        if(i != 0)
        {
            if(sresult.pathInfo.back().sections[i - 1].i - 1 == cur.i)
                add.p_dir = CN_UP_DIR;
            else if(sresult.pathInfo.back().sections[i - 1].i + 1 == cur.i)
                add.p_dir = CN_DOWN_DIR;
            else if(sresult.pathInfo.back().sections[i - 1].j - 1 == cur.j)
                add.p_dir = CN_LEFT_DIR;
            else if(sresult.pathInfo.back().sections[i - 1].j + 1 == cur.j)
                add.p_dir = CN_RIGHT_DIR;
            else
                add.p_dir = CN_NO_DIR;
        }
        else
            add.p_dir = CN_NO_DIR;
        
        if(sresult.pathInfo.back().sections[i + 1].i + 1 == cur.i)
            add.s_dir = CN_DOWN_DIR;
        else if(sresult.pathInfo.back().sections[i + 1].i - 1 == cur.i)
            add.s_dir = CN_UP_DIR;
        else if(sresult.pathInfo.back().sections[i + 1].j + 1 == cur.j)
            add.s_dir = CN_RIGHT_DIR;
        else if(sresult.pathInfo.back().sections[i + 1].j - 1 == cur.j)
            add.s_dir = CN_LEFT_DIR;
        else
            add.s_dir = CN_NO_DIR;
        
        if(ctable[cur.i][cur.j].empty())
            ctable[cur.i][cur.j].push_back(add);
        else
        {
            bool inserted = false;
            for(auto it = ctable[cur.i][cur.j].begin(); it != ctable[cur.i][cur.j].end(); it++)
                if(it->g > cur.g)
                {
                    ctable[cur.i][cur.j].emplace(it, add);
                    inserted = true;
                    break;
                }
            if(!inserted)
                ctable[cur.i][cur.j].push_back(add);
        }
    }
    cur = sresult.pathInfo.back().sections.back();
    if(cur.Parent != NULL)
    {
        if(cur.Parent->i - 1 == cur.i)
            add.p_dir = CN_DOWN_DIR;
        else if(cur.Parent->i + 1 == cur.i)
            add.p_dir = CN_UP_DIR;
        else if(cur.Parent->j - 1 == cur.j)
            add.p_dir = CN_RIGHT_DIR;
        else if(cur.Parent->j + 1 == cur.j)
            add.p_dir = CN_LEFT_DIR;
        else
            add.p_dir = CN_NO_DIR;
    }
    else
        add.p_dir = CN_NO_DIR;
    add.s_dir = CN_GOAL_DIR;
    ctable[cur.i][cur.j].push_back(add);
} 

bool SIPP::findPath(int numOfCurAgent, const cMap &Map)
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
    curNode.g = 0;
    if(ctable[curNode.i][curNode.j].size()==0)
        curNode.interval = {0, CN_INFINITY};
    else
        curNode.interval = {0, ctable[curNode.i][curNode.j][0].g-1};
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
        if(curNode.i == Map.goal_i[numOfCurAgent] && curNode.j == Map.goal_j[numOfCurAgent])
        {
            pathFound = true;
            break;
        }
        std::list<Node> succs;
        succs.clear();
        findSuccessors(curNode, Map, succs, numOfCurAgent);
        std::list<Node>::iterator it = succs.begin();
        auto parent = &(close.find(curNode.i * Map.width + curNode.j)->second);
        while(it != succs.end())
        {
            bool has = false;
            it->Parent = parent;
            auto range = close.equal_range(it->i * Map.width + it->j);
            for(auto i = range.first; i != range.second; i++)
                if(i->second.interval.first <= it->interval.first && i->second.interval.second >= it->interval.second)
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
        for(auto i = hppath.begin(); i != hppath.end(); i++)
        {
            auto j = i;
            j++;
            if(j == hppath.end())
                break;
            if(j->g - i->g != 1)
            {
                Node add = *i;
                add.g++;
                add.F++;
                hppath.emplace(j, add);
                i = hppath.begin();
            }
        }

#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart - begin.QuadPart)/freq.QuadPart;
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
        resultPath.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart - begin.QuadPart)/freq.QuadPart;
#endif
        std::cout<<numOfCurAgent<<" PATH NOT FOUND!\n";
        resultPath.nodescreated = closeSize;
        resultPath.pathfound = false;
        resultPath.path.clear();
        resultPath.sections.clear();
        resultPath.pathlength = 0;
        resultPath.numberofsteps = closeSize;
        sresult.pathfound = false;
        sresult.nodescreated += closeSize;
        sresult.numberofsteps += closeSize;
        sresult.pathInfo[numOfCurAgent] = resultPath;
    }

    return resultPath.pathfound;
}
