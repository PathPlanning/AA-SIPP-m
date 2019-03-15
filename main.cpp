#include"mission.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc==3)
    {
        Mission mission(argv[1], argv[2]);
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

        mission.createSearch();
        mission.createLog();
        mission.startSearch();
        mission.printSearchResultsToConsole();
        mission.saveSearchResultsToLog();
    }
    return 1;
}
