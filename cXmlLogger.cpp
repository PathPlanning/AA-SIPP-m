#include"cXmlLogger.h"

cXmlLogger::cXmlLogger(float loglvl)
{
    loglevel = loglvl;
    LogFileName = "";
    doc = 0;
}

cXmlLogger::~cXmlLogger()
{
    if (doc)
    {
        doc->Clear();
        delete doc;
    }
}

bool cXmlLogger::getLog(const char *FileName)
{
    std::string value;
    TiXmlDocument doc_xml(FileName);

    if(!doc_xml.LoadFile())
    {
        std::cout << "Error opening XML-file in getLog";
        return false;
    }

    value = FileName;
    size_t dotPos = value.find_last_of(".");

    if(dotPos != std::string::npos)
        value.insert(dotPos,CN_LOG);
    else
        value += CN_LOG;

    LogFileName = value;
    doc_xml.SaveFile(LogFileName.c_str());

    doc = new TiXmlDocument(LogFileName.c_str());
    doc->LoadFile();

    TiXmlElement *msg;
    TiXmlElement *root;

    root = doc->FirstChildElement(CNS_TAG_ROOT);
    TiXmlElement *log = new TiXmlElement(CNS_TAG_LOG);
    root->LinkEndChild(log);

    msg = new TiXmlElement(CNS_TAG_MAPFN);
    msg->LinkEndChild(new TiXmlText(FileName));
    log->LinkEndChild(msg);

    msg = new TiXmlElement(CNS_TAG_SUM);
    log->LinkEndChild(msg);

    TiXmlElement* path = new TiXmlElement(CNS_TAG_PATH);
    log->LinkEndChild(path);

    if (loglevel >= CN_LOGLVL_MED)
    {
        TiXmlElement *lowlevel = new TiXmlElement(CNS_TAG_LOWLEVEL);
        log->LinkEndChild(lowlevel);
    }

    return true;
}

void cXmlLogger::saveLog()
{
    if (loglevel == CN_LOGLVL_NO) return;
    doc->SaveFile(LogFileName.c_str());
}

void cXmlLogger::writeToLogSummary(const SearchResult &sresult)
{
    TiXmlElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_SUM);

    unsigned int maxnodes = 0;
    float relPathfound, relAgentSolved = 0;

        int k = 0;
        int j = 0;
        float pathlenght = 0;
        unsigned int totalnodes=0;
        unsigned int numof_pathfound = 0;
        unsigned int num_of_path = 0;
        unsigned int num_of_agents_solved = 0;
        unsigned int num_of_agents = 0;
        unsigned int agent_is_solved = 0;

        for(k = 0; k < sresult.agents; k++)
        {
            agent_is_solved = 0;
            num_of_path += 1;
            if (sresult.pathInfo[k].pathfound)
            {
                numof_pathfound += 1;
                agent_is_solved = 1;

                pathlenght += sresult.pathInfo[k].pathlength;
                totalnodes += sresult.pathInfo[k].nodescreated;
            }
            if (sresult.pathInfo[k].nodescreated > maxnodes)
                maxnodes = sresult.pathInfo[k].nodescreated;
            num_of_agents += 1;
            num_of_agents_solved += agent_is_solved;
        }


        relPathfound = (float)numof_pathfound / num_of_path;
        relAgentSolved = (float)num_of_agents_solved / num_of_agents;

        element->SetDoubleAttribute(CNS_TAG_ATTR_AGENTSSOLVED, relAgentSolved);
        element->SetAttribute(CNS_TAG_ATTR_MAXNODESCR, maxnodes);
        element->SetAttribute(CNS_TAG_ATTR_NODESCREATED, totalnodes);
        element->SetDoubleAttribute(CNS_TAG_ATTR_SUMLENGTH, pathlenght);
        element->SetDoubleAttribute(CNS_TAG_ATTR_AVGLENGTH, pathlenght/num_of_agents_solved);
        element->SetDoubleAttribute(CNS_TAG_ATTR_TIME, sresult.time);
}

void cXmlLogger::writeToLogPath(const SearchResult &sresult)
{
    TiXmlElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);

    TiXmlElement *agent, *summary, *path, *lplevel, *hplevel, *node;

    for(int i = 0; i < sresult.agents; i++)
    {
        agent = new TiXmlElement(CNS_TAG_AGENT);
        agent->SetAttribute(CNS_TAG_ATTR_NUM,i);
        element->LinkEndChild(agent);
        std::list<Node>::const_iterator iterNode;

        summary = new TiXmlElement(CNS_TAG_SUM);
        if (sresult.pathInfo[i].pathfound)
            summary->SetAttribute(CNS_TAG_ATTR_SOLVED,CNS_TAG_ATTR_TRUE);
        else
            summary->SetAttribute(CNS_TAG_ATTR_SOLVED,CNS_TAG_ATTR_FALSE);
        summary->SetAttribute(CNS_TAG_ATTR_PATHSFOUND, sresult.pathInfo[i].pathfound);
        summary->SetAttribute(CNS_TAG_ATTR_MAXNODESCR, sresult.pathInfo[i].nodescreated);
        summary->SetDoubleAttribute(CNS_TAG_ATTR_SUMLENGTH, sresult.pathInfo[i].pathlength);
        summary->SetDoubleAttribute(CNS_TAG_ATTR_AVGLENGTH, sresult.pathInfo[i].pathlength);
        summary->SetDoubleAttribute(CNS_TAG_ATTR_TIME,sresult.pathInfo[i].time);

        agent->LinkEndChild(summary);

        path = new TiXmlElement(CNS_TAG_PATH);
        path->SetAttribute(CNS_TAG_ATTR_NUM,0);

        if(sresult.pathInfo[i].pathfound)
        {
            path->SetAttribute(CNS_TAG_ATTR_PATHFOUND,CNS_TAG_ATTR_TRUE);
            path->SetDoubleAttribute(CNS_TAG_ATTR_LENGTH,sresult.pathInfo[i].pathlength);
        }
        else
        {
            path->SetAttribute(CNS_TAG_ATTR_PATHFOUND,CNS_TAG_ATTR_FALSE);
            path->SetAttribute(CNS_TAG_ATTR_LENGTH,0);
        }
        path->SetAttribute(CNS_TAG_ATTR_NODESCREATED,sresult.pathInfo[i].nodescreated);
        path->SetDoubleAttribute(CNS_TAG_ATTR_TIME,sresult.pathInfo[i].time);
        agent->LinkEndChild(path);

        if (loglevel >= 1 &&  sresult.pathInfo[i].pathfound)
        {
            int k = 0;

            hplevel = new TiXmlElement(CNS_TAG_HPLEVEL);
            path->LinkEndChild(hplevel);
            k = 0;
            auto iter = sresult.pathInfo[i].sections.begin();
            auto it = sresult.pathInfo[i].sections.begin();
            int partnumber=0;
            TiXmlElement *part;
            part = new TiXmlElement(CNS_TAG_SECTION);
            part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
            part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
            part->SetAttribute(CNS_TAG_ATTR_SY, it->i);
            part->SetAttribute(CNS_TAG_ATTR_FX, iter->j);
            part->SetAttribute(CNS_TAG_ATTR_FY, iter->i);
            part->SetDoubleAttribute(CNS_TAG_ATTR_LENGTH, iter->g);
            hplevel->LinkEndChild(part);
            partnumber++;
            while(iter != --sresult.pathInfo[i].sections.end())
            {
                part = new TiXmlElement(CNS_TAG_SECTION);
                part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
                part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
                part->SetAttribute(CNS_TAG_ATTR_SY, it->i);
                iter++;
                part->SetAttribute(CNS_TAG_ATTR_FX, iter->j);
                part->SetAttribute(CNS_TAG_ATTR_FY, iter->i);
                part->SetDoubleAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
                hplevel->LinkEndChild(part);
                it++;
                partnumber++;
            }

            if (loglevel == CN_LOGLVL_LOW)
            {
                lplevel = new TiXmlElement(CNS_TAG_LPLEVEL);
                path->LinkEndChild(lplevel);

                for(iterNode = sresult.pathInfo[i].path.begin();
                    iterNode != sresult.pathInfo[i].path.end(); iterNode++)
                {
                    node = new TiXmlElement(CNS_TAG_NODE);
                    node->SetAttribute(CNS_TAG_ATTR_NUM,k);
                    node->SetAttribute(CNS_TAG_ATTR_X,(*iterNode).j);
                    node->SetAttribute(CNS_TAG_ATTR_Y,(*iterNode).i);
                    lplevel->LinkEndChild(node);
                    k++;
                }
            }
        }
    }
}

void cXmlLogger::writeToLogMap(const cMap &map, const SearchResult &sresult)
{
    std::string text;
    int *curLine;
    curLine = new int[map.width];
    for(int i = 0; i < map.width; i++)
        curLine[i] = 0;
    TiXmlElement *element = doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_PATH);
    TiXmlElement* msg;
    std::string Value;
    std::stringstream stream;

    for(int i=0; i<map.height; i++)
    {
        msg = new TiXmlElement(CNS_TAG_ROW);
        msg->SetAttribute(CNS_TAG_ATTR_NUM, i);
        text = "";
        std::list<Node>::const_iterator iter;
        for(int k = 0; k < sresult.agents; k++)
            for(iter = sresult.pathInfo[k].path.begin(); iter != sresult.pathInfo[k].path.end(); iter++)
            {
                if((*iter).i == i)
                    curLine[(*iter).j] = 1;
            }

        for(int j = 0; j < map.width; j++)
            if(curLine[j] != 1)
            {
                stream << map.Grid[i][j];
                stream >> Value;
                stream.clear();
                stream.str("");
                text = text + Value + " ";
            }
            else
            {	text = text + "*" + " ";
                curLine[j] = 0;
            }

        msg->LinkEndChild(new TiXmlText(text.c_str()));
        element->LinkEndChild(msg);
    }
    delete [] curLine;
}
