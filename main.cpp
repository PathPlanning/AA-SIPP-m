#include"mission.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc==2)
    {
        Mission mission(argv[1]);
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

        mission.createSearch();
        mission.createLog();
        mission.startSearch();
        mission.printSearchResultsToConsole();
        mission.saveSearchResultsToLog();
    }
    return 1;
}
