#include"xmlLogger.h"
using namespace tinyxml2;

XmlLogger::XmlLogger(float loglvl)
{
    loglevel = loglvl;
    LogFileName = "";
    doc = 0;
}

XmlLogger::~XmlLogger()
{
    if (doc)
    {
        doc->Clear();
        delete doc;
    }
}

bool XmlLogger::getLog(const char *FileName)
{
    if (loglevel == CN_LOGLVL_NO)
        return true;

    std::string value;
    XMLDocument doc_xml;

    if(doc_xml.LoadFile(FileName) != XMLError::XML_SUCCESS)
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

    doc = new XMLDocument;
    doc->LoadFile(LogFileName.c_str());

    XMLElement *msg;
    XMLElement *root;

    root = doc->FirstChildElement(CNS_TAG_ROOT);
    XMLElement *log = doc->NewElement(CNS_TAG_LOG);
    root->LinkEndChild(log);

    msg = doc->NewElement(CNS_TAG_MAPFN);
    msg->LinkEndChild(doc->NewText(FileName));
    log->LinkEndChild(msg);

    msg = doc->NewElement(CNS_TAG_SUM);
    log->LinkEndChild(msg);

    XMLElement* path = doc->NewElement(CNS_TAG_PATH);
    log->LinkEndChild(path);

    return true;
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
    element = element->FirstChildElement(CNS_TAG_SUM);

    float pathlenght(0);
    unsigned int maxnodes(0), totalnodes(0);
    for(int k = 0; k < sresult.pathInfo.size(); k++)
        if(sresult.pathInfo[k].pathfound)
        {
            pathlenght += sresult.pathInfo[k].pathlength;
            totalnodes += sresult.pathInfo[k].nodescreated;
            if (sresult.pathInfo[k].nodescreated > maxnodes)
                maxnodes = sresult.pathInfo[k].nodescreated;
        }

    element->SetAttribute(CNS_TAG_ATTR_TRIES, sresult.tries);
    element->SetAttribute(CNS_TAG_ATTR_AGENTSSOLVED, (std::to_string(float(sresult.agentsSolved*100)/sresult.agents)+"%").c_str());
    element->SetAttribute(CNS_TAG_ATTR_MAXNODESCR, maxnodes);
    element->SetAttribute(CNS_TAG_ATTR_TOTALNODES, totalnodes);
    element->SetAttribute(CNS_TAG_ATTR_FLOWTIME, pathlenght);
    element->SetAttribute(CNS_TAG_ATTR_AVGLENGTH, pathlenght/sresult.agentsSolved);
    element->SetAttribute(CNS_TAG_ATTR_MAKESPAN, sresult.makespan);
    element->SetAttribute(CNS_TAG_ATTR_TIME, sresult.time);
}

void XmlLogger::writeToLogPath(const SearchResult &sresult)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    XMLElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    XMLElement *agent, *path;

    for(int i = 0; i < sresult.agents; i++)
    {
        agent = doc->NewElement(CNS_TAG_AGENT);
        agent->SetAttribute(CNS_TAG_ATTR_NUM,i);
        element->LinkEndChild(agent);
        path = doc->NewElement(CNS_TAG_PATH);

        if(sresult.pathInfo[i].pathfound)
        {
            path->SetAttribute(CNS_TAG_ATTR_PATHFOUND, CNS_TAG_ATTR_TRUE);
            path->SetAttribute(CNS_TAG_ATTR_LENGTH, sresult.pathInfo[i].pathlength);
        }
        else
        {
            path->SetAttribute(CNS_TAG_ATTR_PATHFOUND, CNS_TAG_ATTR_FALSE);
            path->SetAttribute(CNS_TAG_ATTR_LENGTH, 0);
        }
        path->SetAttribute(CNS_TAG_ATTR_NODES, sresult.pathInfo[i].nodescreated);
        path->SetAttribute(CNS_TAG_ATTR_TIME, sresult.pathInfo[i].time);
        agent->LinkEndChild(path);

        if (sresult.pathInfo[i].pathfound)
        {
            auto iter = sresult.pathInfo[i].sections.begin();
            auto it = sresult.pathInfo[i].sections.begin();
            int partnumber(0);
            XMLElement *part;
            part = doc->NewElement(CNS_TAG_SECTION);
            part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
            part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
            part->SetAttribute(CNS_TAG_ATTR_SY, it->i);
            part->SetAttribute(CNS_TAG_ATTR_FX, iter->j);
            part->SetAttribute(CNS_TAG_ATTR_FY, iter->i);
            part->SetAttribute(CNS_TAG_ATTR_LENGTH, iter->g);
            path->LinkEndChild(part);
            partnumber++;
            while(iter != --sresult.pathInfo[i].sections.end())
            {
                part = doc->NewElement(CNS_TAG_SECTION);
                part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
                part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
                part->SetAttribute(CNS_TAG_ATTR_SY, it->i);
                iter++;
                part->SetAttribute(CNS_TAG_ATTR_FX, iter->j);
                part->SetAttribute(CNS_TAG_ATTR_FY, iter->i);
                part->SetAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
                path->LinkEndChild(part);
                it++;
                partnumber++;
            }
        }
    }
}

void XmlLogger::writeToLogMap(const Map &map, const SearchResult &sresult)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    std::string text;
    std::vector<int> curLine(map.width, 0);
    XMLElement *element = doc->FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_PATH);
    XMLElement *msg;

    for(int i = 0; i < map.height; i++)
    {
        msg = doc->NewElement(CNS_TAG_ROW);
        msg->SetAttribute(CNS_TAG_ATTR_NUM, i);
        text.clear();
        std::list<Node>::const_iterator iter;
        for(int k = 0; k < sresult.agents; k++)
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
    }
}
