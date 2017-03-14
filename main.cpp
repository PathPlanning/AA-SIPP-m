#include"cMission.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc==2)
    {
        cMission Mission(argv[1]);
        if (!Mission.getConfig())
            return 0;
        else
            std::cout<<"CONFIG LOADED\n";

        if (!Mission.getMap())
        {
            std::cout<<"Program terminated.\n";
            return 0;
        }
        else
            std::cout<<"MAP LOADED\n";

        Mission.createSearch();
        Mission.createLog();
        Mission.startSearch();
        Mission.printSearchResultsToConsole();
        Mission.saveSearchResultsToLog();
    }
    return 1;
}
