#include "sipp.h"

SIPP::~SIPP()
{
}

SIPP::SIPP(double weight, int metrictype)
{
    this->weight = weight;
    this->metrictype = metrictype;
    closeSize = 0;
    openSize = 0;
}

bool SIPP::stopCriterion()
{
    if(openSize == 0)
    {
        //std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }
    return false;
}

double SIPP::countHValue(int i, int j, int goal_i, int goal_j)
{
    return abs(i - goal_i) + abs(j - goal_j);
}

void SIPP::findSuccessors(const Node curNode, const Map &map, std::list<Node> &succs, int numOfCurAgent)
{
    Node newNode;

    for(int i = -1; i <= +1; i++)
    {
        for(int j = -1; j <= +1; j++)
        {
            if(abs(i+j) == 1 && map.CellOnGrid(curNode.i + i, curNode.j + j) && (map.CellIsTraversable(curNode.i + i, curNode.j + j)))
            {
                newNode.i = curNode.i + i;
                newNode.j = curNode.j + j;
                newNode.g = curNode.g + 1;
                double h_value = weight*countHValue(newNode.i, newNode.j, map.agents[numOfCurAgent].goal_i, map.agents[numOfCurAgent].goal_j);
                newNode.F = newNode.g + h_value;
                succs.push_back(newNode);
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
                if (newNode.g > iter->g)
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
        if (iter->j == newNode.j)
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


SearchResult SIPP::startSearch(Map &map)
{
#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    sresult.pathInfo.resize(0);
    sresult.agents = map.agents.size();
    sresult.agentsSolved = 0;
    sresult.flowlength = 0;
    sresult.makespan = 0;
    sresult.pathlength = 0;
    sresult.maxdist = 0;
    ctable.resize(map.height);
    std::vector<double> dists(map.agents.size(), -1);
    for(int i = 0; i < map.agents.size(); i++)
        dists[i] = sqrt(pow(map.agents[i].start_i - map.agents[i].goal_i, 2) + pow(map.agents[i].start_j - map.agents[i].goal_j, 2));
    int k = map.agents.size() - 1;
    current_priorities.resize(map.agents.size());
    while(k >= 0)
    {
        double mindist = CN_INFINITY;
        int min_i = -1;
        for(unsigned int i = 0; i < dists.size(); i++)
            if(mindist > dists[i])
            {
                min_i = i;
                mindist = dists[i];
            }
        current_priorities[min_i] = min_i;
        dists[min_i] = CN_INFINITY;
        k--;
    }
    for(int i = 0; i < map.height; i++)
    {
        ctable[i].resize(map.width);
        for(int j = 0; j < map.width; j++)
            ctable[i][j].resize(0);
    }
    for(int i = 0; i < map.agents.size(); i++)
    {
        map.addConstraint(map.agents[i].start_i, map.agents[i].start_j);
        map.addConstraint(map.agents[i].goal_i, map.agents[i].goal_j);
    }
    for(int numOfCurAgent = 0; numOfCurAgent < map.agents.size(); numOfCurAgent++)
    {
        map.removeConstraint(map.agents[current_priorities[numOfCurAgent]].start_i, map.agents[current_priorities[numOfCurAgent]].start_j);
        map.removeConstraint(map.agents[current_priorities[numOfCurAgent]].goal_i, map.agents[current_priorities[numOfCurAgent]].goal_j);
        findPath(current_priorities[numOfCurAgent], map);
        map.addConstraint(map.agents[current_priorities[numOfCurAgent]].goal_i, map.agents[current_priorities[numOfCurAgent]].goal_j);
        map.addConstraint(map.agents[current_priorities[numOfCurAgent]].start_i, map.agents[current_priorities[numOfCurAgent]].start_j);
        for(int i = 0; i < map.height; i++)
            open[i].clear();
        open.clear();
        close.clear();
    }
#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.time = static_cast<double>(end.QuadPart - begin.QuadPart)/freq.QuadPart;
#endif

    repairPaths(map);

#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.reptime = static_cast<double>(end.QuadPart - begin.QuadPart)/freq.QuadPart - sresult.time;
#endif

    std::vector<conflict> conflicts = CheckConflicts();
    //for(unsigned int i = 0; i < conflicts.size(); i++)
    //    std::cout<< i << " " << conflicts[i].agent1 << " " << conflicts[i].agent2 << " " << conflicts[i].g << "\n";
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
    for(unsigned int i = 0; i < sresult.agents; i++)
        for(unsigned int j = i + 1; j < sresult.agents; j++)
            for(unsigned int k = 0; k < sresult.pathInfo[i].sections.size(); k++)
                for(unsigned int l = 0; l < sresult.pathInfo[j].sections.size(); l++)
                {

                    cur = sresult.pathInfo[i].sections[k];
                    check = sresult.pathInfo[j].sections[l];
                    if(cur.i == check.i && cur.j == check.j && cur.g==check.g)
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
                        /*if(((cur.i == checknext.i && cur.j == checknext.j && cur.g==checknext.g) ||
                            (curnext.i == check.i && curnext.j == check.j && curnext.g == check.g)) &&
                                ((cur.i-curnext.i)!=(check.i-checknext.i) || (cur.j-curnext.j)!=(check.j-checknext.j)))
                        {
                            conf.agent1 = i;
                            conf.agent2 = j;
                            conf.sec1 = k;
                            conf.sec2 = l;
                            conf.i = cur.i;
                            conf.j = cur.j;
                            conf.g = cur.g;
                            conflicts.push_back(conf);
                        }*/
                    }
                }
    return conflicts;
}

void SIPP::addConstraints(std::vector<Node> sections)
{
    Node cur;
    movement add;
    for(unsigned int i = 0; i < sections.size() - 1; i++)
    {
        cur = sections[i];
        add.g = cur.g;
        if(i != 0)
        {
            if(sections[i - 1].i - 1 == cur.i)
                add.p_dir = CN_UP_DIR;
            else if(sections[i - 1].i + 1 == cur.i)
                add.p_dir = CN_DOWN_DIR;
            else if(sections[i - 1].j - 1 == cur.j)
                add.p_dir = CN_LEFT_DIR;
            else if(sections[i - 1].j + 1 == cur.j)
                add.p_dir = CN_RIGHT_DIR;
            else
                add.p_dir = CN_NO_DIR;
        }
        else
            add.p_dir = CN_NO_DIR;
        
        if(sections[i + 1].i + 1 == cur.i)
            add.s_dir = CN_DOWN_DIR;
        else if(sections[i + 1].i - 1 == cur.i)
            add.s_dir = CN_UP_DIR;
        else if(sections[i + 1].j + 1 == cur.j)
            add.s_dir = CN_RIGHT_DIR;
        else if(sections[i + 1].j - 1 == cur.j)
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
    cur = sections.back();
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
    add.g = sections.back().g;
    ctable[cur.i][cur.j].push_back(add);
} 

void SIPP::repairPaths(const Map &map)
{

    close.clear();
    std::vector<Node> sections = sresult.pathInfo[0].sections;
    addConstraints(sections);
    sresult.makespan = 0;
    sresult.flowlength = 0;
    sresult.pathlength = 0;
    for(int i=1; i<sresult.agents; i++)
    {
        sections = sresult.pathInfo[i].sections;
        std::vector<Node> path;
        path.push_back(sections[0]);
        for(int k=1; k<sections.size(); k++)
        {
            Node cur = sections[k];
            bool ok = true;
            if(k != 0)
            {
                cur.g = std::max(cur.g, path.back().g+1);
                int dir;
                if(path.back().i - 1 == cur.i)
                    dir = CN_DOWN_DIR;
                else if(path.back().i + 1 == cur.i)
                    dir = CN_UP_DIR;
                else if(path.back().j - 1 == cur.j)
                    dir = CN_RIGHT_DIR;
                else if(path.back().j + 1 == cur.j)
                    dir = CN_LEFT_DIR;
                else
                    dir = CN_NO_DIR;
                for(int n = 0; n < ctable[cur.i][cur.j].size(); n++)
                {
                    if(fabs(ctable[cur.i][cur.j][n].g - cur.g) < 1.5)
                    {
                        if(ctable[cur.i][cur.j][n].g == cur.g)
                        {
                            sections[k-1].g+=1.0;
                            path.pop_back();
                            k-=2;
                            ok=false;
                            break;
                        }
                        else if(ctable[cur.i][cur.j][n].g - cur.g == 1.0)
                        {
                            if(abs(dir - ctable[cur.i][cur.j][n].p_dir) == 2)
                            {
                                sections[k-1].g+=1.0;
                                path.pop_back();
                                k-=2;
                                ok=false;
                                break;
                            }
                        }
                        else if(ctable[cur.i][cur.j][n].g - cur.g == -1.0)
                        {
                            if(abs(dir - ctable[cur.i][cur.j][n].s_dir) == 2)
                            {
                                sections[k-1].g+=1.0;
                                path.pop_back();
                                k-=2;
                                ok=false;
                                break;
                            }
                        }
                    }
                    else if(ctable[cur.i][cur.j][n].g > cur.g + 1.0)
                        break;
                }
            }
            if(ok)
                path.push_back(cur);
        }
        std::vector<Node> secs;
        Node cur = path[0];
        cur.g = 0;
        if(path[0].g != 0)
            path.insert(path.begin(),cur);
        secs.push_back(cur);
        for(int k=1; k<path.size(); k++)
        {
            if(path[k].g != path[k-1].g+1.0)
            {
                cur = path[k];
                cur.g = path[k-1].g;
                while(cur.g+1 < path[k].g)
                {
                    cur.g+=1.0;
                    secs.push_back(cur);
                }
            }
            secs.push_back(path[k]);
        }
        addConstraints(secs);
        sresult.pathInfo[i].sections = secs;
        sresult.makespan = std::max(sresult.makespan, secs.back().g);
        sresult.pathlength += secs.back().g;
        sresult.flowlength += abs(secs.begin()->i - secs.back().i) + abs(secs.begin()->j - secs.back().j);
    }
}

bool SIPP::findPath(int numOfCurAgent, const Map &map)
{
#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    open.resize(map.height);

    ResultPathInfo resultPath;
    openSize = 0;
    closeSize = 0;

    Node curNode(map.agents[numOfCurAgent].start_i, map.agents[numOfCurAgent].start_j, 0, 0);
    curNode.g = 0;
    if(ctable[curNode.i][curNode.j].empty())
        curNode.interval = {0, CN_INFINITY};
    else
        curNode.interval = {0, ctable[curNode.i][curNode.j][0].g-1};
    bool pathFound = false;
    open[curNode.i].push_back(curNode);
    openSize++;
    while(!stopCriterion())
    {
        curNode = findMin(map.height);
        open[curNode.i].pop_front();
        openSize--;
        close.insert({curNode.i * map.width + curNode.j, curNode});
        closeSize++;
        if(curNode.i == map.agents[numOfCurAgent].goal_i && curNode.j == map.agents[numOfCurAgent].goal_j)
        {
            pathFound = true;
            break;
        }
        std::list<Node> succs;
        succs.clear();
        findSuccessors(curNode, map, succs, numOfCurAgent);
        std::list<Node>::iterator it = succs.begin();
        auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
        while(it != succs.end())
        {
            it->Parent = parent;
            if(close.find(it->i*map.width + it->j) == close.end())
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
        double length=0;
        for(unsigned int i = 1; i < hppath.size(); i++)
            length += countHValue(hppath[i].i, hppath[i].j, hppath[i-1].i, hppath[i-1].j);
        sresult.flowlength += length;
        sresult.maxdist = std::max(sresult.maxdist, length);
        resultPath.sections = hppath;
        //makeSecondaryPath(curNode);
        resultPath.nodescreated = openSize + closeSize;
        resultPath.pathfound = true;
        resultPath.path = lppath;
        resultPath.numberofsteps = closeSize;
        resultPath.pathlength = curNode.g;
        sresult.pathfound = true;
        sresult.pathlength += curNode.g;
        sresult.nodescreated += openSize + closeSize;
        sresult.numberofsteps += closeSize;
        sresult.makespan = std::max(sresult.makespan, resultPath.pathlength);
        sresult.pathInfo.push_back(resultPath);
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
        //std::cout<<numOfCurAgent<<" PATH NOT FOUND!\n";
        resultPath.nodescreated = closeSize;
        resultPath.pathfound = false;
        resultPath.path.clear();
        resultPath.sections.clear();
        resultPath.pathlength = 0;
        resultPath.numberofsteps = closeSize;
        sresult.pathfound = false;
        sresult.nodescreated += closeSize;
        sresult.numberofsteps += closeSize;
        sresult.pathInfo.push_back(resultPath);
    }

    return resultPath.pathfound;
}
