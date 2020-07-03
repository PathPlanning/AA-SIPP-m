#include"xmlLogger.h"
using namespace tinyxml2;

bool XmlLogger::createLog(const char *FileName)
{
    if (loglevel == CN_LOGLVL_NO)
        return true;

    std::string value(FileName);
    size_t dotPos = value.find_last_of(".");
    if(dotPos != std::string::npos)
        value.insert(dotPos,CN_LOG);
    else
        value += CN_LOG;
    LogFileName = value;

    std::ofstream out(LogFileName);
    out<<"<?xml version=\"1.0\" ?>\n<root>\n</root>";
    out.close();
    doc = new XMLDocument;
    doc->LoadFile(LogFileName.c_str());
    if(!doc)
        return false;
    XMLElement *root = doc->FirstChildElement(CNS_TAG_ROOT);
    root->LinkEndChild(doc->NewElement(CNS_TAG_LOG));

    return true;
}

void XmlLogger::writeToLogInput(const char *taskName, const char *mapName, const char *configName, const char *obstaclesName)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    else if(loglevel == CN_LOGLVL_PRIMS || loglevel == CN_LOGLVL_POINTS || loglevel == CN_LOGLVL_BOTH)
    {
        XMLElement *log = doc->FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG);
        XMLElement *element = doc->NewElement(CNS_TAG_TASKFN);
        element->LinkEndChild(doc->NewText(taskName));
        log->LinkEndChild(element);
        element = doc->NewElement(CNS_TAG_MAPFN);
        element->LinkEndChild(doc->NewText(mapName));
        log->LinkEndChild(element);
        element = doc->NewElement(CNS_TAG_CONFIGFN);
        element->LinkEndChild(doc->NewText(configName));
        log->LinkEndChild(element);
        if(obstaclesName)
        {
            element = doc->NewElement(CNS_TAG_OBSFN);
            element->LinkEndChild(doc->NewText(obstaclesName));
            log->LinkEndChild(element);
        }
    }
    else if(loglevel == CN_LOGLVL_ALL)
    {
        if(taskName == mapName)//i.e. all_in_one
        {
            writeToLogFile(taskName);
        }
        else
        {
            if(obstaclesName && obstaclesName != mapName)
                writeToLogFile(obstaclesName);
            writeToLogFile(configName);
            writeToLogFile(mapName);
            writeToLogFile(taskName);
        }
    }

}

void XmlLogger::writeToLogFile(const char *fileName)
{
    XMLDocument file;
    file.LoadFile(fileName);
    XMLNode *prev = nullptr;
    for(XMLNode* node = file.RootElement()->FirstChild(); node; node = node->NextSibling())
    {
        XMLNode *clone = node->DeepClone(doc);
        if(!prev)
            doc->RootElement()->InsertFirstChild(clone);
        else
            doc->RootElement()->InsertAfterChild(prev, clone);
        prev = clone;
    }
}

void XmlLogger::saveLog()
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    doc->SaveFile(LogFileName.c_str());
}

void XmlLogger::writeToLogSummary(const SearchResult &sresult)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    XMLElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element->LinkEndChild(doc->NewElement(CNS_TAG_SUM));
    element = element->FirstChildElement(CNS_TAG_SUM);
    element->SetAttribute(CNS_TAG_ATTR_RUNTIME, float(sresult.runtime));
    element->SetAttribute(CNS_TAG_ATTR_TRIES, sresult.tries);
    element->SetAttribute(CNS_TAG_ATTR_AGENTSSOLVED, ((std::to_string(sresult.agentsSolved) + " (" + std::to_string(float(sresult.agentsSolved*100)/sresult.agents)+"%)")).c_str());
    element->SetAttribute(CNS_TAG_ATTR_FLOWTIME, float(sresult.flowtime));
    element->SetAttribute(CNS_TAG_ATTR_MAKESPAN, float(sresult.makespan));
}

void XmlLogger::writeToLogPath(const SearchResult &sresult, const Task &task, const Config &config)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    XMLElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    XMLElement *agent_elem, *path;
    for(unsigned int i = 0; i < task.getNumberOfAgents(); i++)
    {
        Agent agent = task.getAgent(i);
        agent_elem = doc->NewElement(CNS_TAG_AGENT);
        agent_elem->SetAttribute(CNS_TAG_ATTR_ID, agent.id.c_str());
        agent_elem->SetAttribute(CNS_TAG_ATTR_SX, agent.start_j);
        agent_elem->SetAttribute(CNS_TAG_ATTR_SY, agent.start_i);
        agent_elem->SetAttribute(CNS_TAG_ATTR_GX, agent.goal_j);
        agent_elem->SetAttribute(CNS_TAG_ATTR_GY, agent.goal_i);
        agent_elem->SetAttribute(CNS_TAG_ATTR_SIZE, float(agent.size));
        element->LinkEndChild(agent_elem);
        path = doc->NewElement(CNS_TAG_PATH);

        if(sresult.pathInfo[i].pathfound)
        {
            path->SetAttribute(CNS_TAG_ATTR_PATHFOUND, CNS_TAG_ATTR_TRUE);
            path->SetAttribute(CNS_TAG_ATTR_RUNTIME, float(sresult.pathInfo[i].runtime));
            path->SetAttribute(CNS_TAG_ATTR_DURATION, float(sresult.pathInfo[i].pathlength));
        }
        else
        {
            path->SetAttribute(CNS_TAG_ATTR_PATHFOUND, CNS_TAG_ATTR_FALSE);
            path->SetAttribute(CNS_TAG_ATTR_RUNTIME, float(sresult.pathInfo[i].runtime));
            path->SetAttribute(CNS_TAG_ATTR_DURATION, 0);
        }
        agent_elem->LinkEndChild(path);
        if (sresult.pathInfo[i].pathfound)
        {
            if(loglevel == CN_LOGLVL_PRIMS || loglevel == CN_LOGLVL_BOTH || loglevel == CN_LOGLVL_ALL)
            {
                auto it = sresult.pathInfo[i].primitives.begin();
                int partnumber(0);
                XMLElement *part;
                while(it != --sresult.pathInfo[i].primitives.end())
                {
                    part = doc->NewElement("action");
                    part->SetAttribute("number", partnumber);
                    part->SetAttribute("primitive_id", it->id);
                    part->SetAttribute("x0", it->source.j);
                    part->SetAttribute("y0", it->source.i);
                    part->SetAttribute("xf", it->target.j);
                    part->SetAttribute("yf", it->target.i);
                    part->SetAttribute("t0", it->begin);
                    part->SetAttribute("duration", it->duration);
                    path->LinkEndChild(part);
                    it++;
                    partnumber++;
                }
            }
            if(loglevel == CN_LOGLVL_POINTS || loglevel == CN_LOGLVL_BOTH || loglevel == CN_LOGLVL_ALL)
            {
                auto it = sresult.pathInfo[i].points.begin();
                int partnumber(0);
                XMLElement *part;
                while(it != --sresult.pathInfo[i].points.end())
                {
                    part = doc->NewElement("terminal_point");
                    part->SetAttribute("x", it->j);
                    part->SetAttribute("y", it->i);
                    part->SetAttribute("angle", it->angle);
                    part->SetAttribute("primitive_id", it->primitive_id);
                    part->SetAttribute("T", it->t);
                    path->LinkEndChild(part);
                    it++;
                    partnumber++;
                }
            }
        }
    }
}

void XmlLogger::writeToLogMap(const Map &map, const SearchResult &sresult)
{
    //if (loglevel == CN_LOGLVL_NO)
        return;
    /*std::string text;
    std::vector<int> curLine(map.width, 0);
    XMLElement *element = doc->FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG);
    element->LinkEndChild(doc->NewElement(CNS_TAG_PATH));
    element = element->FirstChildElement(CNS_TAG_PATH);
    XMLElement *msg;

    for(int i = 0; i < map.height; i++)
    {
        msg = doc->NewElement(CNS_TAG_ROW);
        msg->SetAttribute(CNS_TAG_ATTR_NUM, i);
        text.clear();
        std::list<Node>::const_iterator iter;
        for(unsigned int k = 0; k < sresult.agents; k++)
            for(iter = sresult.pathInfo[k].path.begin(); iter != sresult.pathInfo[k].path.end(); iter++)
                if((*iter).i == i)
                    curLine[(*iter).j] = 1;

        for(int j = 0; j < map.width; j++)
            if(curLine[j] != 1)
                text += std::to_string(map.Grid[i][j]) + " ";
            else
            {
                text += "* ";
                curLine[j] = 0;
            }
        msg->LinkEndChild(doc->NewText(text.c_str()));
        element->LinkEndChild(msg);
    }*/
}
