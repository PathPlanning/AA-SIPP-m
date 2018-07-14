#include "aa_sipp.h"

AA_SIPP::AA_SIPP(double weight, int rescheduling, int timelimit, int prioritization)
{
    this->weight = weight;
    this->rescheduling = rescheduling;
    if(timelimit > 0)
        this->timelimit = timelimit;
    else
        this->timelimit = CN_INFINITY;
    this->prioritization = prioritization;
    closeSize = 0;
    openSize = 0;
    constraints = nullptr;
}

AA_SIPP::~AA_SIPP()
{
}

bool AA_SIPP::stopCriterion()
{
    if(openSize == 0)
    {
        std::cout << "OPEN list is empty! ";
        return true;
    }
    return false;
}

double AA_SIPP::getCost(int a_i, int a_j, int b_i, int b_j)
{
    return sqrt((a_i - b_i) * (a_i - b_i) + (a_j - b_j) * (a_j - b_j));
}

double AA_SIPP::calcHeading(const Node &node, const Node &son)
{
    double heading = acos((node.i - son.i)/getCost(son.i, son.j, node.i, node.j))*180/PI;
    if(heading > 180)
        heading = 360 - heading;
    return heading;
}

void AA_SIPP::findSuccessors(const Node curNode, const Map &map, std::list<Node> &succs, int numOfCurAgent)
{
    Node newNode;
    auto parent = &(close.find(curNode.i*map.width + curNode.j)->second);
    for(int i = -1; i <= +1; i++)
    {
        for(int j = -1; j <= +1; j++)
        {
            if(((i == 0 && j != 0) || (i != 0 && j == 0)) && map.CellOnGrid(curNode.i + i, curNode.j + j) && map.CellIsTraversable(curNode.i + i, curNode.j + j))
            {
                newNode.i = curNode.i + i;
                newNode.j = curNode.j + j;
                newNode.g = curNode.g + 1.0;
                newNode.F = newNode.g + weight*getCost(newNode.i, newNode.j, map.agents[numOfCurAgent].goal_i, map.agents[numOfCurAgent].goal_j);
                newNode.Parent = parent;
                newNode = resetParent(newNode, curNode, map);
                succs.push_front(newNode);
            }
        }
    }
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
    if (open[newNode.i].size() == 0)
    {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }

    std::list<Node>::iterator iter, pos, delpos;
    bool posFound = false;
    pos = open[newNode.i].end();
    for(iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
    {
        if ((iter->F >= newNode.F) && (!posFound))
        {
            if (fabs(iter->F - newNode.F) < CN_EPSILON)//CN_EPSILON is needed to prevent mistakes with comparison of double-type values
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
            if((iter->g - newNode.g) < CN_EPSILON)
                return;
            else
            {
                if(pos == iter)
                {
                    iter->F = newNode.F;
                    iter->g = newNode.g;
                    iter->interval = newNode.interval;
                    iter->Parent = newNode.Parent;
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

void AA_SIPP::setPriorities(const Map& map)
{
    current_priorities.clear();
    current_priorities.resize(map.agents.size(), -1);
    if(prioritization == CN_IP_FIFO)
        for(int i = 0; i < map.agents.size(); i++)
            current_priorities[i] = i;
    else  if(prioritization != CN_IP_RANDOM)
    {
        std::vector<double> dists(map.agents.size(), -1);
        for(int i = 0; i < map.agents.size(); i++)
            dists[i] = sqrt(pow(map.agents[i].start_i - map.agents[i].goal_i, 2) + pow(map.agents[i].start_j - map.agents[i].goal_j, 2));
        int k = map.agents.size() - 1;
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
            if(prioritization == CN_IP_LONGESTF)
                current_priorities[k] = min_i;
            else
                current_priorities[map.agents.size() - k - 1] = min_i;
            dists[min_i] = CN_INFINITY;
            k--;
        }
    }
    else //random
    {
        for(int i = 0; i < map.agents.size(); i++)
            current_priorities[i] = i;
        std::mt19937 g(rand());
        std::shuffle(current_priorities.begin(), current_priorities.end(), g);
    }
}

bool AA_SIPP::changePriorities(int bad_i)
{
    if(rescheduling == CN_RE_NO)
        return false;

    priorities.push_back(current_priorities);
    if(rescheduling == CN_RE_RULED) //rises the piority of the agent that can't find its path
    {
        for(auto it = current_priorities.begin(); it != current_priorities.end(); it++)
            if(*it == bad_i)
            {
                current_priorities.erase(it);
                current_priorities.insert(current_priorities.begin(), bad_i);
                break;
            }
        for(unsigned int i = 0; i < priorities.size(); i++)
            for(unsigned int j = 0; j < priorities[i].size(); j++)
            {
                if(j + 1 == priorities[i].size())
                    return false;
                if(current_priorities[j] != priorities[i][j])
                    break;
            }
        return true;
    }
    else //random
    {
        std::mt19937 g(rand());
        std::shuffle(current_priorities.begin(),current_priorities.end(), g);
        bool unique = false;
        int maxtries(1), tries(0);
        for(unsigned int i = 1; i <= current_priorities.size(); i++)
            maxtries *= i;
        while(!unique && tries < maxtries)
        {
            tries++;
            for(unsigned int i = 0; i < priorities.size(); i++)
            {
                for(unsigned int j = 0; j < priorities[i].size(); j++)
                {
                    if(j + 1 == priorities[i].size())
                        unique = false;
                    if(current_priorities[j] != priorities[i][j])
                        break;
                }
                if(!unique)
                {
                    std::shuffle(current_priorities.begin(),current_priorities.end(), g);
                    break;
                }
            }
            unique = true;
        }
        return unique;
    }
}

SearchResult AA_SIPP::startSearch(Map &map)
{

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    int tries(0), bad_i(0);
    bool solution_found(false);
    double timespent(0);
    priorities.clear();
    open.resize(map.height);
    setPriorities(map);
    do
    {
        constraints = new VelocityConstraints(map.width, map.height);
        sresult.pathInfo.clear();
        sresult.agents = map.agents.size();
        sresult.agentsSolved = 0;
        sresult.pathlength = 0;
        sresult.makespan = 0;
        sresult.flowlength = 0;
        sresult.maxdist = 0;
        for(int i = 0; i < map.agents.size(); i++)
        {
            map.addConstraint(map.agents[i].start_i, map.agents[i].start_j);
            map.addConstraint(map.agents[i].goal_i, map.agents[i].goal_j);
        }
        for(int numOfCurAgent = 0; numOfCurAgent < map.agents.size(); numOfCurAgent++)
        {
            map.agents[current_priorities[numOfCurAgent]].size = sqrt(2.0)/4.0;
            constraints->setSize(map.agents[current_priorities[numOfCurAgent]].size);
            map.removeConstraint(map.agents[current_priorities[numOfCurAgent]].start_i, map.agents[current_priorities[numOfCurAgent]].start_j);
            map.removeConstraint(map.agents[current_priorities[numOfCurAgent]].goal_i, map.agents[current_priorities[numOfCurAgent]].goal_j);
            if(!findPath(current_priorities[numOfCurAgent], map))
            {
                bad_i = current_priorities[numOfCurAgent];
                break;
            }
            map.addConstraint(map.agents[current_priorities[numOfCurAgent]].start_i, map.agents[current_priorities[numOfCurAgent]].start_j);
            map.addConstraint(map.agents[current_priorities[numOfCurAgent]].goal_i, map.agents[current_priorities[numOfCurAgent]].goal_j);

            if(numOfCurAgent + 1 == map.agents.size())
                solution_found = true;
        }
#ifdef __linux__
        gettimeofday(&end, NULL);
        timespent = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        timespent = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        sresult.time = timespent;
        repairPaths(map);
#ifdef __linux__
        gettimeofday(&end, NULL);
        timespent = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        timespent = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        sresult.reptime = timespent - sresult.time;
        delete constraints;
        tries++;

        if(timespent > timelimit)
            break;
    } while(changePriorities(bad_i) && !solution_found);
#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
    sresult.tries = tries;
    if(!sresult.pathfound)
    {
        std::vector<conflict> confs = CheckConflicts();
        for(unsigned int i = 0; i < confs.size(); i++)
            std::cout<<confs[i].i<<" "<<confs[i].j<<" "<<confs[i].g<<" "<<confs[i].agent1<<" "<<confs[i].agent2<<"\n";
    }
    return sresult;
}


Node AA_SIPP::resetParent(Node current, Node Parent, const Map &map)
{
    if(Parent.Parent == nullptr || (current.i == Parent.Parent->i && current.j == Parent.Parent->j))
        return current;
    if(lineofsight->checkLine(Parent.Parent->i, Parent.Parent->j, current.i, current.j, map))
    {
        current.g = Parent.Parent->g + getCost(Parent.Parent->i, Parent.Parent->j, current.i, current.j);
        current.Parent = Parent.Parent;
    }
    return current;
}

void AA_SIPP::repairPaths(Map &map)
{
    close.clear();
    constraints->addConstraints(sresult.pathInfo.at(0).sections, sqrt(2.0)/4);
    sresult.pathlength = sresult.pathInfo[0].sections.back().g;
    sresult.flowlength = getCost(sresult.pathInfo[0].sections.begin()->i,sresult.pathInfo[0].sections.begin()->j,
            sresult.pathInfo[0].sections.back().i, sresult.pathInfo[0].sections.back().j);
    for(int i = 1; i < sresult.agents; i++)
    {
        auto sections = sresult.pathInfo.at(i).sections;
        sresult.flowlength += getCost(sections.begin()->i, sections.begin()->j, sections.back().i, sections.back().j);
        sections[0].interval = {0, CN_INFINITY};
        std::vector<std::vector<std::tuple<double, double, double>>> allintervals(sections.size());
        std::vector<std::pair<double, double>> intervals;
        std::vector<std::tuple<double, double, double>> temp;
        temp.push_back(std::make_tuple(0, CN_INFINITY, 0));
        allintervals[0]=temp;
        bool update=false;
        for(int k = 1; k < sections.size(); k++)
        {
            if(update)
            {
                update = false;
                allintervals[k-1].erase(allintervals[k-1].begin());
                if(allintervals[k-1].empty())
                {
                    update=true;
                    k-=2;
                    continue;
                }
                sections[k-1].interval.first = std::get<0>(allintervals[k-1][0]);
                sections[k-1].interval.second = std::get<1>(allintervals[k-1][0]);
                sections[k-1].g = std::get<2>(allintervals[k-1][0]);
                k--;
                continue;
            }
            sections[k].Parent = &sections[k-1];
            Node cur = sections[k];
            cur.g = sections[k-1].g + getCost(cur.i, cur.j, sections[k-1].i, sections[k-1].j);
            std::vector<double> EAT;
            intervals = constraints->findIntervals(cur, EAT, close, map.width);
            if(intervals.empty())
            {
                allintervals[k-1].erase(allintervals[k-1].begin());
                if(allintervals[k-1].empty())
                {
                    update=true;
                    k-=2;
                    continue;
                }
                sections[k-1].interval.first = std::get<0>(allintervals[k-1][0]);
                sections[k-1].interval.second = std::get<1>(allintervals[k-1][0]);
                sections[k-1].g = std::get<2>(allintervals[k-1][0]);
                k--;
            }
            else
            {
                temp.clear();
                for(int l = 0; l < intervals.size(); l++)
                {
                    auto tup = std::make_tuple(intervals[l].first, intervals[l].second, EAT[l]);
                    if(std::find(allintervals[k].begin(), allintervals[k].end(),tup) == allintervals[k].end())
                        allintervals[k].push_back(tup);
                }
                for(int l = 0; l < intervals.size(); l++)
                {
                    if(cur.g <= EAT[l])
                    {
                        if(cur.g != EAT[l])
                            for(int m = k; m < sections.size(); m++)
                                sections[m].g += EAT[l] - cur.g;
                        sections[k].interval = intervals[l];
                        sections[k].g = EAT[l];
                        break;
                    }
                }
            }
        }
        for(unsigned int k = 1; k < sections.size(); k++)
            if((sections[k].g - (sections[k - 1].g + getCost(sections[k].i, sections[k].j, sections[k - 1].i, sections[k - 1].j))) > CN_EPSILON)
            {
                Node add = sections[k - 1];
                add.Parent = sections[k].Parent;
                add.g = sections[k].g - getCost(sections[k].i, sections[k].j, sections[k - 1].i, sections[k - 1].j);
                sections.emplace(sections.begin() + k, add);
                sections[k+1].Parent = &sections[k];
                k++;
            }
        constraints->addConstraints(sections, sqrt(2.0)/4);
        sresult.pathlength += sections.back().g;
        sresult.pathInfo.at(i).sections = sections;
        sresult.makespan = std::max(sresult.makespan, sections.back().g);
    }
}


bool AA_SIPP::findPath(int numOfCurAgent, const Map &map)
{

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    close.clear();
    for(unsigned int i = 0; i< open.size(); i++)
        open[i].clear();
    ResultPathInfo resultPath;
    openSize = 0;
    closeSize = 0;
    curagent = map.agents[numOfCurAgent];
    lineofsight = new LineOfSight(curagent.size);
    Node curNode(curagent.start_i, curagent.start_j, 0, weight * getCost(curNode.i, curNode.j, curagent.goal_i, curagent.goal_j));
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
        if(curNode.i == curagent.goal_i && curNode.j == curagent.goal_j)
        {
            pathFound = true;
            break;
        }
        std::list<Node> successors;
        successors.clear();
        findSuccessors(curNode, map, successors, numOfCurAgent);
        for(auto it = successors.begin(); it != successors.end(); it++)
            if(close.find(it->i*map.width+it->j) == close.end())
            {
                addOpen(*it);
            }
    }
    if(pathFound)
    {
        makePrimaryPath(curNode);
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        resultPath.sections = hppath;
        //makeSecondaryPath(curNode);

        double length=0;
        for(unsigned int i = 1; i < hppath.size(); i++)
            length += getCost(hppath[i].i, hppath[i].j, hppath[i-1].i, hppath[i-1].j);
        sresult.flowlength += length;
        sresult.maxdist = std::max(sresult.maxdist, length);
        resultPath.nodescreated = openSize + closeSize;
        resultPath.pathfound = true;
        resultPath.path = lppath;
        resultPath.numberofsteps = closeSize;
        resultPath.pathlength = curNode.g;
        sresult.pathfound = true;
        sresult.pathlength += curNode.g;
        sresult.nodescreated += openSize + closeSize;
        sresult.numberofsteps += closeSize;
        sresult.makespan = std::max(sresult.makespan, curNode.g);
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
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        std::cout<<"Path for agent "<<numOfCurAgent<<" not found!\n";
        sresult.pathfound = false;
        sresult.nodescreated += closeSize;
        sresult.numberofsteps += closeSize;
        resultPath.nodescreated = closeSize;
        resultPath.pathfound = false;
        resultPath.path.clear();
        resultPath.sections.clear();
        resultPath.pathlength = 0;
        resultPath.numberofsteps = closeSize;
        sresult.pathInfo.push_back(resultPath);
    }
    return resultPath.pathfound;
}

std::vector<conflict> AA_SIPP::CheckConflicts()
{
    std::vector<conflict> conflicts(0);
    conflict conf;
    Node cur, check;
    std::vector<std::vector<conflict>> positions;
    positions.resize(sresult.agents);
    for(unsigned int i = 0; i < sresult.agents; i++)
    {
        if(!sresult.pathInfo[i].pathfound)
            continue;
        positions[i].resize(0);
        int k = 0;
        double part = 1;
        for(unsigned int j = 1; j < sresult.pathInfo[i].sections.size(); j++)
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
            if(curg <= cur.g)
            {
                positions[i].push_back(conf);
                k++;
            }
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
    unsigned int max = 0;
    for(unsigned int i = 0; i < positions.size(); i++)
        if(positions[i].size() > max)
            max = positions[i].size();
    for(unsigned int i = 0; i < sresult.agents; i++)
    {
        for(unsigned int k = 0; k < max; k++)
        {
            for(unsigned int j = i + 1; j < sresult.agents; j++)
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
                if(sqrt((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j)) + CN_EPSILON < sqrt(2.0)/2.0)
                {
                    std::cout<<a.i<<" "<<a.j<<" "<<b.i<<" "<<b.j<<" "<<sqrt((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j))<<"\n";
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
    if(curNode.Parent != nullptr)
    {
        curNode = *curNode.Parent;
        if(curNode.Parent != nullptr)
        {
            do
            {
                path.push_front(curNode);
                curNode = *curNode.Parent;
            }
            while(curNode.Parent != nullptr);
        }
        path.push_front(curNode);
    }
    for(auto it = path.begin(); it != path.end(); it++)
        hppath.push_back(*it);
    return;
}

void AA_SIPP::makeSecondaryPath(Node curNode)
{
    lppath.clear();
    if(curNode.Parent != nullptr)
    {
        std::vector<Node> lineSegment;
        do
        {
            calculateLineSegment(lineSegment, *curNode.Parent, curNode);
            lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
            curNode = *curNode.Parent;
        }
        while(curNode.Parent != nullptr);
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
