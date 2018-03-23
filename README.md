# AA-SIPP(m)

## Description
AA-SIPP(m) is a pathplanning algorithm that builds conflict-free any-angle trajectories on grids for a set of agents. It's a prioritized planner, e.g. all agents are assigned unique priorities and paths are planned one by one in accordance with the imposed ordering using AA-SIPP (any-angle SIPP) algorithm. The latter builds on top of the SIPP planner, which is tailored to path finding for a single agent moving amidst static and dynamic obstacles (other agents in this case).

Algorithm supports XML files as input and output format. Input file contains map, coordinates of start and goal locations of all agents as well as the parameters of the algorithm (see __"Input and Output files"__ or [examples](https://github.com/PathPlanning/AA-SIPP-m/tree/master/Instances))

## Getting Started

To go and try this algorithm you can use QtCreator or CMake.
Both `.pro` and `CMakeLists` files are available in the repository.

Notice, that project uses C++11 standart. Make sure that your compiler supports it.

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

**[Qt Creator](https://info.qt.io/download-qt-for-device-creation?hsCtaTracking=c80600ba-f2ea-45ed-97ef-6949c1c4c236%7C643bd8f4-2c59-4c4c-ba1a-4aaa05b51086)**  &mdash; a cross-platform C++, JavaScript and QML integrated development environment which is part of the SDK for the Qt GUI Application development framework.

**[CMake](https://cmake.org/)** &mdash; an open-source, cross-platform family of tools designed to build, test and package software.

### Installing

Download current repository to your local machine. Use
```
git clone https://github.com/PathPlanning/AA-SIPP-m.git
```
or direct downloading.

Built current project using **Qt Creator** or **CMake**. To launch the compiled file you will need to pass input XML file as an argument. Output file for this project will be placed in the same folder as input file and, by default, will be named `_log.xml`. For examlpe, using CMake
```bash
cd PATH_TO_THE_PROJECT
cmake .
make
./AA-SIPP-m initial_file_name.xml
```
Output file for this project will be placed in the same folder as input file and, by default, will be named `_log.xml`. For examlpe,
```
"initial_file_name.xml" -> "initial_file_name_log.xml"
```
## Input and Output files

Both files are an XML file with a specific structure.

Note that all tags in XML files are case sensitive.

Input file should contain:

* Mandatory tag `<map>`. It describes the environment.
    * `<height>` and `<width>` &mdash; mandatory tags that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, (*width* - 1, *height* - 1) is lower right.
    * `<agents>` &mdash; mandatory tag that defines the number of agents which contains the instance.
    * `<startx>` and `<starty>` &mdash; mandatory tags that define horizontal (X) and vertical (Y) offset of the start location from the upper left corner. Legal values for *startx* are [0, .., *width* - 1], for *starty* - [0, .., *height* - 1].
    * `<finishx>` and `<finishy>` &mdash; mandatory tags that horizontal (X) and vertical (Y) offset of the goal location.
    * Number of `<startx>`, `<starty>`, `<finishx>` and `<finishy>` tags should be equal to the number of agents, as each of them defines the start(goal) location only for one agent.
    * `<grid>` &mdash; mandatory tag that describes the square grid constituting the map. It consists of `<row>` tags. Each `<row>` contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" &mdash; for not traversable (actually any other figure but "0" can be used instead of "1").
    
* Mandatory tag `<algorithm>`. It describes the parameters of the algorithm.

    * `<hweight>` or `<weight>` &mdash; defines the weight of heuristic function. Default value is "1".
    * `<allowanyangle>` &mdash; possible values `true` or `false`. Defines the choice between AA-SIPP and SIPP algorithms.
    * `<prioritization>` &mdash; defines the initial prioitization of the agents. Possible values: `fifo` - priority of agents corresponds to the order of their enumeration in XML file; `shortest_first` - the less the distance between the start and goal locations, the higher the priority of the agent; `longest_first` - the more the distance between the start and goal locations, the higher the priority of the agent; `random` - shuffles the priorities of all agents in a random way.
    * `<rescheduling>` &mdash; defines the possibility of using rescheduling in cases when the algorithm fails to find a solution. Possible values: `no` - rescheduling is disabled; `rulebased` - rises the priority of failed agent to the top; `random` - shuffles the priorities of all agents in a random way.
    * `<timelimit>` &mdash; defines  the amount of time that the algorithm can spend on finding a solution. Can be helpful in cases of using rescheduling. Possible values: `-1` - no limit; `n` - number of seconds (n>0).
    * `<startsafeinterval>` &mdash; defines the size of additional constraints in the start locations of low-prioirity agents. Helps to find a solution for instances with many agents without rescheduling. Possible values: `0` - no startsafeintervals; `n` - the size of constraints, counts in conditional time units.
    * `<constraintstype>` &mdash; defines the procedure that the algorithm uses to count EAT. Possible values: `point` - the procedure based on splitting the found trajectories of high-priority agents into sequences of point-constraints (original procedure introduced in ICAPS-2017 paper); `velocity` - procedure based on `time-to-collision` formula (the fastest); `section` - procedure that considers all possible cases of intersecting of two sections and counts EAT precisely (produces the best quality solutions).
    * `<turningweight>` &mdash; defines the cost of changing heading (the moving direction). Need for AAt-SIPP(m). Value `1` means that the turning on 180 degrees requires the same amount of time as for moving between the centers of two adjacent cells (1 conditional time unit).

* Optional tag `<options>`. Options that are not related to search.

    * `<loglevel>` &mdash; defines the level of detalization of log-file. Default value is "1". Possible values:
        - "0" &mdash; log-file is not created.
        - "1" &mdash; all the input data is copied to the log-file plus short `<summary>` is appended. `<summary>` contains info of the path length, number of steps, elapsed time, etc. It also contains `<path>` tag. It looks like `<grid>` but cells forming the path are marked by "\*" instead of "0". Moreover, the log contains the information about each agent and its path.
    * `<logpath>` - defines the directory where the log-file should be written. If not specified directory of the input file is used.
    * `<logname>` - defines the name of log-file. If not specified the name of the log file is: "input file name" + "\_log" + input file extension.

## Repository folders

`Videos` folder contains a few video demonstrations of how AA-SIPP(m) works.

`Instances` folder contains an example of input and output files. It also contains the collections of instances that were used in corresponding conference papers for experimental evaluation.

[![Build Status](https://travis-ci.org/PathPlanning/AA-SIPP-m.svg?branch=master)](https://travis-ci.org/PathPlanning/AA-SIPP-m)
