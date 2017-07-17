#include"cMission.h"

cMission::cMission(const char* fName)
{
    m_fileName = fName;
    m_pSearch = 0;
    m_pLogger = 0;
}
cMission::~cMission()
{
    delete m_pSearch;
    delete m_pLogger;
}

bool cMission::getMap()
{
    return m_map.getMap(m_fileName);
}

bool cMission::getConfig()
{
    return m_config.getConfig(m_fileName);
}

void cMission::createSearch()
{
    if(m_pSearch)
    {
        delete m_pSearch;
        delete m_pLogger;
    }
    if(m_config.searchParams[CN_PT_AA]==0)
        m_pSearch = new SIPP(m_config.searchParams[CN_PT_WEIGHT], m_config.searchParams[CN_PT_MT], m_config.searchParams[CN_PT_BT]);
    else
        m_pSearch = new AA_SIPP(m_config.searchParams[CN_PT_WEIGHT], m_config.searchParams[CN_PT_BT]);
}

bool cMission::createLog()
{
    if(m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_LOW || m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_HIGH || m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_MED)
    {
        m_pLogger = new cXmlLogger(m_config.searchParams[CN_PT_LOGLVL]);
    }
    else if(m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_NO)
    {
        m_pLogger = new cXmlLogger(m_config.searchParams[CN_PT_LOGLVL]);
        return true;
    }
    else
    {
        std::cout<<"'loglevel' is not correctly specified in input XML-file.\n";
        return false;
    }
    return m_pLogger->getLog(m_fileName);
}

void cMission::startSearch()
{
    std::cout<<"SEARCH STARTED\n";
    sr = m_pSearch->startSearch(m_pLogger, m_map);
}

void cMission::printSearchResultsToConsole()
{
    std::cout<<"Solved:"<<(float)sr.agentsSolved*100/sr.agents<<"%  Time:"<<sr.time<<"s Pathlength:"<<sr.pathlength<<"\n";
}

void cMission::saveSearchResultsToLog()
{
    std::cout<<"LOG STARTED\n";
    m_pLogger->writeToLogSummary(sr);
    if (sr.pathfound)
    {
        m_pLogger->writeToLogPath(sr);
        m_pLogger->writeToLogMap(m_map,sr);
    }
    m_pLogger->saveLog();
    std::cout<<"LOG SAVED\n";
}

