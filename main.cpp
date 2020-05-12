#include"mission.h"
#include <iostream>

int main(int argc, char* argv[])
{
    std::string map_name = "map.xml";
    std::string config_name = "config.xml";
    std::string task_name = "task.xml";
    int agents(-1);
    Mission mission;
    mission.setFileNames(task_name.c_str(), map_name.c_str(), config_name.c_str(), nullptr);
    if(argc>=4)
        mission.setFileNames(argv[1], argv[2], argv[3], nullptr);
    mission.getConfig();
    mission.getMap();
    mission.getTask(agents);
    mission.createSearch();
    mission.createLog();
    mission.startSearch();
    mission.printSearchResultsToConsole();
    mission.saveSearchResultsToLog();
    return 0;
}
