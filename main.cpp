#include"mission.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc > 1)
    {
        Mission mission;
        if(argc == 5)
            mission.setFileNames(argv[1], argv[2], argv[3], argv[4]);
        else if(argc == 4)
            mission.setFileNames(argv[1], argv[2], argv[3], nullptr);
        else if(argc == 2)
            mission.setFileNames(argv[1], argv[1], argv[1], argv[1]);
        else
        {
            std::cout<<"Wrong number of input XML-files. It should be either all-in-one file, or three ones: map-file, task-file and config-file.\n";
            return 0;
        }
        if (!mission.getConfig())
            return 0;
        else
            std::cout<<"CONFIG LOADED\n";

        if (!mission.getMap())
        {
            std::cout<<"Program terminated.\n";
            return 0;
        }
        else
            std::cout<<"MAP LOADED\n";
        if (!mission.getTask())
        {
            std::cout<<"Program terminated.\n";
            return 0;
        }
        else
            std::cout<<"TASK LOADED\n";
        if(mission.getObstacles())
        {
            std::cout<<"OBSTACLES LOADED\n";
        }

        mission.createSearch();
        mission.createLog();
        mission.startSearch();
        mission.printSearchResultsToConsole();
        mission.saveSearchResultsToLog();
    }
    return 1;
}
