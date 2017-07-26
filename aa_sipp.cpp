#include "aa_sipp.h"


AA_SIPP::AA_SIPP(double weight, bool breakingties, int constraints_type)
{
    this->weight = weight;
    this->breakingties = breakingties;
    this->constraints_type = constraints_type;
    closeSize = 0;
    openSize = 0;
}

AA_SIPP::~AA_SIPP()
{
    delete constraints;
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
                if(i*j != 0)
                    if(Map.CellIsObstacle(curNode.i, curNode.j + j) || Map.CellIsObstacle(curNode.i + i, curNode.j))
                        continue;
                newNode.i = curNode.i + i;
                newNode.j = curNode.j + j;
                if(i!=0 && j!=0)
                    newNode.g = curNode.g + sqrt(2);
                else
                    newNode.g = curNode.g + 1.0;
                newNode.Parent = parent;
                h_value = weight*calculateDistanceFromCellToCell(newNode.i, newNode.j, Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]);
                intervals = constraints->findIntervals(newNode, EAT, close, Map.width);
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
                    intervals = constraints->findIntervals(newNode, EAT, close, Map.width);
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
            if(error > delta_i)
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
        if(map.CellIsObstacle(i, j + step_j))
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
            if(error > delta_j)
            {
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    if(map.CellIsObstacle(i, j + step_j))
                        return false;
                if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < sep_value)
                    if(map.CellIsObstacle(i + 2*step_i, j))
                        return false;
                i += step_i;
                error -= delta_j;
            }
        }
        if(map.CellIsObstacle(i, j))
            return false;
        if(map.CellIsObstacle(i + step_i, j))
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
            if (fabs(iter->F - newNode.F) < CN_EPSILON)//CN_EPSILON is needed to prevent mistakes with comparison of double-type values
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
            if(iter->g - newNode.g < CN_EPSILON)
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
    /*if(constraints_type == CN_CT_POINT)
        constraints = new PointConstraints(Map.width, Map.height);
    else if(constraints_type == CN_CT_VELOCITY)
        constraints = new VelocityConstraints(Map.width, Map.height);
    else
        constraints = new SectionConstraints(Map.width, Map.height);*/
    constraints = new ComboConstraints(Map.width, Map.height);
    sresult.pathInfo.resize(0);
    sresult.agents = Map.agents;
    sresult.agentsSolved = 0;
    for(int i = 0; i < Map.agents; i++)
    {
        Map.addConstraint(Map.start_i[i], Map.start_j[i]);
        Map.addConstraint(Map.goal_i[i], Map.goal_j[i]);
    }
    constraints->setAgent(1);
    for(int numOfCurAgent = 0; numOfCurAgent < Map.agents; numOfCurAgent++)
    {
        Map.removeConstraint(Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent]);
        Map.removeConstraint(Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]);
        if(numOfCurAgent+1 == Map.agents)
        {
            if(constraints_type == CN_CT_POINT)
                constraints->setAgent(0);
            else if(constraints_type == CN_CT_VELOCITY)
                constraints->setAgent(1);
            else if(constraints_type == CN_CT_SECTION)
                constraints->setAgent(2);
            findPath(numOfCurAgent, Map);
        }
        else if(findPath(numOfCurAgent, Map))
            constraints->addConstraints(sresult.pathInfo.back().sections);
        close.clear();
        for(int i = 0; i< Map.height; i++)
            open[i].clear();
        open.clear();
    }
#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
    std::vector<conflict> confs = CheckConflicts();
    if(confs.size()>0)
        sresult.pathInfo.back().pathlength=-1;
    for(int i = 0; i < confs.size(); i++)
        std::cout<<confs[i].i<<" "<<confs[i].j<<" "<<confs[i].g<<" "<<confs[i].agent1<<" "<<confs[i].agent2<<"\n";
    return sresult;
}


Node AA_SIPP::resetParent(Node current, Node Parent, const cMap &Map)
{
    if(Parent.Parent == NULL || (current.i == Parent.Parent->i && current.j == Parent.Parent->j))
        return current;

    if(lineOfSight(Parent.Parent->i, Parent.Parent->j, current.i, current.j, Map))
    {
        current.g = Parent.Parent->g + calculateDistanceFromCellToCell(Parent.Parent->i, Parent.Parent->j, current.i, current.j);
        current.Parent = Parent.Parent;
    }
    return current;
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
    open.resize(Map.height);
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
        for(auto it = successors.begin(); it != successors.end(); it++)
            addOpen(*it);
    }
    if(pathFound)
    {
        makePrimaryPath(curNode);
        for(int i = 1; i < hppath.size(); i++)
            if((hppath[i].g - (hppath[i - 1].g + calculateDistanceFromCellToCell(hppath[i].i, hppath[i].j, hppath[i - 1].i, hppath[i - 1].j))) > CN_EPSILON)
            {
                Node add = hppath[i - 1];
                add.Parent = hppath[i].Parent;
                add.g = hppath[i].g - calculateDistanceFromCellToCell(hppath[i].i, hppath[i].j, hppath[i - 1].i,hppath[i - 1].j);
                auto parent = (close.insert({add.i*Map.width + add.j, add}));
                hppath[i].Parent = &(*parent).second;
                hppath.emplace(hppath.begin() + i, add);
                i++;
            }
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
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
        sresult.pathInfo.push_back(resultPath);
        sresult.agentsSolved++;
        //if(numOfCurAgent + 1 == Map.agents)
        //    std::cout<<"Time:"<<resultPath.time<<" Nodes:"<<resultPath.nodescreated<<" Pathlength:"<<curNode.g<<"\n";
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
                if(sqrt((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j)) + CN_EPSILON < 1.0)
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
    lppath.clear();
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
