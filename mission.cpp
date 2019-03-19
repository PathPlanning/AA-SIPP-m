#include"mission.h"

Mission::Mission()
{
    m_pSearch = nullptr;
    m_pLogger = nullptr;
}

Mission::~Mission()
{
    delete m_pSearch;
    delete m_pLogger;
}

void Mission::setFileNames(const char *mapName, const char *taskName, const char *configName)
{
    this->mapName = mapName;
    this->taskName = taskName;
    this->configName = configName;
}

bool Mission::getMap()
{
    return m_map.getMap(mapName);
}

bool Mission::getTask()
{
    return (m_task.getTask(taskName) && m_task.validateTask(m_map));
}

bool Mission::getConfig()
{
    return m_config.getConfig(configName);
}

void Mission::createSearch()
{
    if(m_pSearch)
    {
        delete m_pSearch;
        delete m_pLogger;
    }
    m_pSearch = new AA_SIPP(m_config);
}

bool Mission::createLog()
{
    if(m_config.loglevel == CN_LOGLVL_HIGH)
        m_pLogger = new XmlLogger(m_config.loglevel);
    else if(m_config.loglevel != CN_LOGLVL_NO)
    {
        std::cout<<"'loglevel' is not correctly specified in input XML-file.\n";
        return false;
    }
    return m_pLogger->getLog(taskName);
}

void Mission::startSearch()
{
    //std::cout<<"SEARCH STARTED\n";
    sr = m_pSearch->startSearch(m_map, m_task);
}

void Mission::printSearchResultsToConsole()
{
    //std::cout<<bool(sr.agentsSolved/sr.agents)<<" "<<sr.time<<" "<<sr.makespan<<" "<<sr.pathlength<<" "<<sr.flowlength<<"\n";
    std::cout<<"Results:\nTask solved: "<<bool(sr.agentsSolved/sr.agents)<<" \nTries: "<<sr.tries<<" \nPaths found: "<<(float)sr.agentsSolved*100/sr.agents<<"% \nTime: "<<sr.time<<" \nMakespan: "<<sr.makespan<<" \nFlowtime: "<<sr.pathlength<<"\n";
}

void Mission::saveSearchResultsToLog()
{
    if(m_config.loglevel == CN_LOGLVL_NO)
        return;
    std::cout<<"LOG STARTED\n";
    m_pLogger->writeToLogSummary(sr);
    if(sr.pathfound)
    {
        m_pLogger->writeToLogPath(sr);
        m_pLogger->writeToLogMap(m_map,sr);
    }
    m_pLogger->saveLog();
    std::cout<<"LOG SAVED\n";
}

