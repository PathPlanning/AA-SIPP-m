#include"mission.h"

Mission::Mission(const char* fName)
{
    m_fileName = fName;
    m_pSearch = nullptr;
    m_pLogger = nullptr;
}
Mission::~Mission()
{
    delete m_pSearch;
    delete m_pLogger;
}

bool Mission::getMap()
{
    return m_map.getMap(m_fileName);
}

bool Mission::getConfig()
{
    return m_config.getConfig(m_fileName);
}

void Mission::createSearch()
{
    if(m_pSearch)
    {
        delete m_pSearch;
        delete m_pLogger;
    }
    if(m_config.searchParams[CN_PT_AA] == 0)
        m_pSearch = new SIPP(m_config.searchParams[CN_PT_WEIGHT], m_config.searchParams[CN_PT_MT]);
    else
        m_pSearch = new AA_SIPP(m_config.searchParams[CN_PT_WEIGHT], m_config.searchParams[CN_PT_RE],
                                m_config.searchParams[CN_PT_TL], m_config.searchParams[CN_PT_IP]);
}

bool Mission::createLog()
{
    if(m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_HIGH)
        m_pLogger = new XmlLogger(m_config.searchParams[CN_PT_LOGLVL]);
    else if(m_config.searchParams[CN_PT_LOGLVL] != CN_LOGLVL_NO)
    {
        std::cout<<"'loglevel' is not correctly specified in input XML-file.\n";
        return false;
    }
    return m_pLogger->getLog(m_fileName);
}

void Mission::startSearch()
{
    std::cout<<"SEARCH STARTED\n";
    sr = m_pSearch->startSearch(m_map);
}

void Mission::printSearchResultsToConsole()
{
    std::cout<<"Task solved: "<<bool(sr.agentsSolved/sr.agents)<<" \nPlanning Time: "<<sr.time
             <<" \nRepair Time: "<<sr.reptime<<" \nMakespan: "<<sr.makespan<<" \nFlowtime: "<<sr.pathlength
             <<" \nSummary Distance: "<<sr.flowlength<<" \nMax Distance: "<<sr.maxdist<<"\n";
}

void Mission::saveSearchResultsToLog()
{
    if(m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_NO)
        return;
    std::cout<<"LOG STARTED\n";
    m_pLogger->writeToLogSummary(sr);
    if(sr.pathfound)
    {
        m_pLogger->writeToLogPath(sr);
        //m_pLogger->writeToLogMap(m_map,sr);
    }
    m_pLogger->saveLog();
    std::cout<<"LOG SAVED\n";
}

