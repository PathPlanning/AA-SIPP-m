#include "aa_sipp.h"

AA_SIPP::AA_SIPP(const Config &config)
{
    this->config = std::make_shared<const Config> (config);
    openSize = 0;
    constraints = nullptr;
    intervals_time = 0;
    cells_time = 0;
    primitives.loadPrimitives("trajectories_moving.xml");
}

AA_SIPP::~AA_SIPP()
{
}

bool AA_SIPP::stopCriterion(const Node &curNode, Node &goalNode)
{
    if(Open.isEmpty())
    {
        std::cout << "OPEN list is empty! ";
        return true;
    }
    if(curNode.i == curagent.goal_i && curNode.j == curagent.goal_j && curNode.interval.end == CN_INFINITY && curNode.speed == 0)
    {
        goalNode = curNode;
        return true;
    }
    return false;
}

double AA_SIPP::getHValue(int i, int j)
{
    return (sqrt(pow(i - curagent.goal_i, 2) + pow(j - curagent.goal_j, 2)));
}

std::list<Node> AA_SIPP::findSuccessors(const Node curNode, const Map &map)
{
    Node newNode;
    std::list<Node> successors;
    std::vector<double> EAT;
    std::vector<SafeInterval> intervals;
    double h_value;
    auto parent = &(*closed.get<0>().find(boost::make_tuple(curNode.i, curNode.j, curNode.interval_id, curNode.angle_id, curNode.speed)));
    std::vector<Primitive> prims = primitives.getPrimitives(curNode.i, curNode.j, curNode.angle_id, curNode.speed, map);
    for(auto p:prims)
    {
        newNode = Node(curNode.i + p.target.i, curNode.j + p.target.j);
        newNode.angle_id = p.target.angle_id;
        newNode.speed = p.target.speed;
        newNode.primitive = p;
        newNode.heading = p.target.angle_id;
        newNode.primitive.begin = curNode.g;
        newNode.primitive.setSize(curagent.size);
        newNode.primitive.setSource(curNode.i, curNode.j);
        newNode.g = curNode.g + p.duration;
        newNode.Parent = parent;
        h_value = getHValue(newNode.i, newNode.j);
        if(newNode.i == curNode.i && newNode.j == curNode.j)
        {
            if(curNode.speed == 0 && curNode.interval.end > newNode.g)
            {
                newNode.interval = curNode.interval;
                newNode.interval_id = curNode.interval_id;
                newNode.angle_id = p.target.angle_id;
                newNode.F = newNode.g + h_value;
                if(closed.get<0>().find(boost::make_tuple(newNode.i, newNode.j, newNode.interval_id, newNode.angle_id, newNode.speed)) == closed.get<0>().end())
                    successors.push_front(newNode);
            }
        }
        else
        {
            intervals = constraints->findIntervals(newNode, EAT, closed, Open);
            for(unsigned int k = 0; k < intervals.size(); k++)
            {
                newNode.interval = intervals[k];
                newNode.interval_id = newNode.interval.id;
                newNode.g = EAT[k];
                newNode.F = newNode.g + h_value;
                newNode.angle_id = p.target.angle_id;
                successors.push_front(newNode);
            }
        }
    }
    return successors;
}

void AA_SIPP::setPriorities(const Task& task)
{
    current_priorities.clear();
    current_priorities.resize(task.getNumberOfAgents(), -1);
    if(config->initialprioritization == CN_IP_FIFO)
        for(int i = 0; i < task.getNumberOfAgents(); i++)
            current_priorities[i] = i;
    else if(config->initialprioritization != CN_IP_RANDOM)
    {
        std::vector<double> dists(task.getNumberOfAgents(), -1);
        for(int i = 0; i < task.getNumberOfAgents(); i++)
            dists[i] = sqrt(pow(task.getAgent(i).start_i - task.getAgent(i).goal_i, 2) + pow(task.getAgent(i).start_j - task.getAgent(i).goal_j, 2));
        int k = task.getNumberOfAgents() - 1;
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
            if(config->initialprioritization == CN_IP_LONGESTF)
                current_priorities[k] = min_i;
            else
                current_priorities[task.getNumberOfAgents() - k - 1] = min_i;
            dists[min_i] = CN_INFINITY;
            k--;
        }
    }
    else //random
    {
        for(int i = 0; i < task.getNumberOfAgents(); i++)
            current_priorities[i] = i;
        std::mt19937 g(rand());
        std::shuffle(current_priorities.begin(), current_priorities.end(), g);
    }
}

bool AA_SIPP::changePriorities(int bad_i)
{
    if(config->rescheduling == CN_RE_NO)
        return false;
    priorities.push_back(current_priorities);
    if(config->rescheduling == CN_RE_RULED) //rises the piority of the agent that can't find its path
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
        int maxtries(1000), tries(0);
        while(!unique && tries < maxtries)
        {
            tries++;
            unique = true;
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
        }
        return unique;
    }
}

SearchResult AA_SIPP::startSearch(Map &map, Task &task, DynamicObstacles &obstacles)
{

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    bool solution_found(false);
    int tries(0), bad_i(0);
    double timespent(0);
    priorities.clear();
    setPriorities(task);
    do
    {
        constraints = new Constraints(map.width, map.height);
        constraints->setObstacles(&obstacles);
        for(int k = 0; k < obstacles.getNumberOfObstacles(); k++)
            constraints->addConstraints(obstacles.getPrimitives(k), obstacles.getSize(k), obstacles.getMSpeed(k), map);

        sresult.pathInfo.clear();
        sresult.pathInfo.resize(task.getNumberOfAgents());
        sresult.agents = task.getNumberOfAgents();
        sresult.agentsSolved = 0;
        sresult.flowtime = 0;
        sresult.makespan = 0;

        for(unsigned int numOfCurAgent = 0; numOfCurAgent < task.getNumberOfAgents(); numOfCurAgent++)
        {
            curagent = task.getAgent(current_priorities[numOfCurAgent]);
            constraints->setSize(curagent.size);
            if(findPath(current_priorities[numOfCurAgent], map))
                constraints->addConstraints(sresult.pathInfo[current_priorities[numOfCurAgent]].primitives, curagent.size, curagent.mspeed, map);
            else
            {
                bad_i = current_priorities[numOfCurAgent];
                break;
            }
            if(numOfCurAgent + 1 == task.getNumberOfAgents())
                solution_found = true;
        }


        delete constraints;
        tries++;
#ifdef __linux__
    gettimeofday(&end, NULL);
    timespent = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    timespent = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        if(timespent > config->timelimit)
            break;
    } while(changePriorities(bad_i) && !solution_found);


#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.runtime = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
    sresult.tries = tries;
    return sresult;
}

bool AA_SIPP::findPath(unsigned int numOfCurAgent, const Map &map)
{

    Open.clear();
    closed.clear();
    ResultPathInfo resultPath;
    openSize = 0;
    constraints->resetSafeIntervals(map.width, map.height);
    for(int i=0; i<map.height; i++)
        for(int j=0; j<map.width; j++)
            constraints->updateCellSafeIntervals({i,j});
    //constraints->updateCellSafeIntervals({curagent.start_i, curagent.start_j});

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    Node curNode(curagent.start_i, curagent.start_j, 0, 0), goalNode(curagent.goal_i, curagent.goal_j, CN_INFINITY, CN_INFINITY);
    curNode.F = getHValue(curNode.i, curNode.j);
    curNode.interval = constraints->getSafeInterval(curNode.i, curNode.j, 0);
    curNode.interval_id = curNode.interval.id;
    curNode.heading = curagent.start_heading;
    curNode.angle_id = 0;
    curNode.speed = 0;
    curNode.primitive.id = -1;
    curNode.primitive.source.angle_id = 0;
    curNode.primitive.target.angle_id = 0;
    Open.addOpen(curNode);
    while(!stopCriterion(curNode, goalNode))
    {
        curNode = Open.findMin();
        closed.insert(curNode);
        for(Node s:findSuccessors(curNode, map))
            Open.addOpen(s);
    }
    if(goalNode.g < CN_INFINITY)
    {
        makePrimaryPath(goalNode);
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.runtime = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        makeSecondaryPath();
        std::cout<<"PATH FOUND\n";
        resultPath.points = point_path;
        resultPath.pathfound = true;
        resultPath.primitives = primitives_path;
        resultPath.pathlength = goalNode.g;
        sresult.pathfound = true;
        sresult.flowtime += goalNode.g;
        sresult.makespan = std::max(sresult.makespan, goalNode.g);
        sresult.pathInfo[numOfCurAgent] = resultPath;
        sresult.agentsSolved++;
    }
    else
    {
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.runtime = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        std::cout<<"Path for agent "<<curagent.id<<" not found!\n";
        sresult.pathfound = false;
        resultPath.pathfound = false;
        resultPath.points.clear();
        resultPath.primitives.clear();
        resultPath.pathlength = 0;
        sresult.pathInfo[numOfCurAgent] = resultPath;
    }
    return resultPath.pathfound;
}

void AA_SIPP::makePrimaryPath(Node curNode)
{
    primitives_path.clear();
    std::vector<Node> path;
    path.push_back(curNode);
    if(curNode.Parent != nullptr)
    {
        curNode = *curNode.Parent;
        if(curNode.Parent != nullptr)
        {
            do
            {
                path.push_back(curNode);
                curNode = *curNode.Parent;
            }
            while(curNode.Parent != nullptr);
        }
        path.push_back(curNode);
    }
    std::reverse(path.begin(), path.end());

    for(unsigned int i = 1; i < path.size(); i++)
    {
        if(path[i].g - (path[i - 1].g + path[i].primitive.duration) > CN_RESOLUTION)
        {
            Node add = path[i - 1];
            add.Parent = path[i].Parent;
            add.g = path[i].g - path[i].primitive.duration;
            add.angle_id = path[i-1].angle_id;
            path.emplace(path.begin() + i, add);
            i++;
        }
    }
    for(int i = 1; i < path.size(); i++)
    {
        Node cur, next;
        cur = path[i-1];
        next = path[i];
        if(cur.i == next.i && cur.j == next.j && cur.angle_id == next.angle_id)
        {
            Primitive wait;
            wait.source.i = cur.i;
            wait.source.j = cur.j;
            wait.source.angle_id = cur.primitive.target.angle_id;
            wait.target.i = cur.i;
            wait.target.j = cur.j;
            wait.id = 0;
            wait.target.angle_id = cur.primitive.target.angle_id;
            wait.begin = cur.g;
            wait.duration = next.g - cur.g;
            wait.cells = {Cell(cur.i, cur.j)};
            wait.cells[0].interval = {0, wait.duration}; //not sure...
            wait.type = -2;
            path[i+1].primitive.begin = next.g;
            primitives_path.push_back(wait);

        }
        else
            primitives_path.push_back(next.primitive);
    }
    return;
}

void AA_SIPP::makeSecondaryPath()
{
    point_path.clear();
    int i(0);
    double t(0);
    while(i < primitives_path.size())
    {
        Primitive cur = primitives_path[i];
        while(t < cur.begin + cur.duration)
        {
            auto p = cur.getPos(t-cur.begin);
            p.i = cur.source.i+p.i;
            p.j = cur.source.j+p.j;
            double angle = cur.getAngle(t-cur.begin)*180/PI;
            if(cur.type == -2)//wait action
            {
                cur.id=0;
                angle = cur.source.angle_id*45;
            }
            else if(cur.type == -1)//rotation action
            {
                angle = cur.source.angle_id*45 + (cur.target.angle_id - cur.source.angle_id)*45*(t-cur.begin)/cur.duration;
            }
            TerminalPoint n(p.i, p.j, t, angle, cur.id);
            point_path.push_back(n);
            t+=0.1;
        }
        i++;
    }
    TerminalPoint n(curagent.goal_i, curagent.goal_j, primitives_path.back().begin + primitives_path.back().duration, 180 - primitives_path.back().target.angle_id*45, primitives_path.back().id);
    point_path.push_back(n);
    return;
}
