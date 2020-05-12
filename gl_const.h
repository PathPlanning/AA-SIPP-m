#ifndef GL_CONST_H
#define GL_CONST_H

//constants
#define PI                  3.14159265359
#define CN_EPSILON          1e-9
#define CN_INFINITY         1e+9
#define CN_LOG              "_log"
#define CN_OBSTL            1

//default values
#define CN_DEFAULT_SIZE                     0.5
#define CN_DEFAULT_RSPEED                   1.0
#define CN_DEFAULT_MSPEED                   1.0
#define CN_DEFAULT_SHEADING                 0
#define CN_DEFAULT_GHEADING                 -1 //i.e. whatever
#define CN_DEFAULT_RESCHEDULING             CN_RE_NO
#define CN_DEFAULT_INITIALPRIORITIZATION    CN_IP_FIFO
#define CNS_DEFAULT_INITIALPRIORITIZATION   CNS_IP_FIFO
#define CN_DEFAULT_LOGLVL                   CN_LOGLVL_NORM
#define CNS_DEFAULT_LOGLVL                  CNS_LOGLVL_NORM
#define CN_DEFAULT_TIMELIMIT                CN_INFINITY
#define CNS_DEFAULT_TIMELIMIT               "'infinity' (no limit)"
#define CN_DEFAULT_CONNECTEDNESS            2
#define CN_DEFAULT_ALLOWANYANGLE            true
#define CNS_DEFAULT_ALLOWANYANGLE           "true"
#define CN_DEFAULT_PLANFORTURNS             false
#define CNS_DEFAULT_PLANFORTURNS            "false"
#define CN_DEFAULT_ADDITIONALWAIT           0
#define CN_DEFAULT_STARTSAFEINTERVAL        0
#define CN_DEFAULT_INFLATEINTERVALS          0

#define CN_HEADING_WHATEVER                 -1
#define CNS_HEADING_WHATEVER                "whatever"

//loglevel
#define CN_LOGLVL_NO	0
#define CN_LOGLVL_NORM	1
#define CN_LOGLVL_ALL   2

#define CNS_LOGLVL_NO	"none"
#define CNS_LOGLVL_NORM	"regular"
#define CNS_LOGLVL_FULL "full"

//initial prioritization
#define CN_IP_COST       1
#define CN_IP_DISTANCE   2
#define CN_IP_RANDOM     3
#define CN_IP_FIFO       4

#define CNS_IP_COST      "cost"
#define CNS_IP_DISTANCE  "distance"
#define CNS_IP_RANDOM    "random"
#define CNS_IP_FIFO      "fifo"

//rescheduling
#define CN_RE_NO         1
#define CN_RE_RULED      2
#define CN_RE_RANDOM     3

#define CNS_RE_NO        "no"
#define CNS_RE_RULED     "rulebased"
#define CNS_RE_RANDOM    "random"

/*
 * XML file tags ---------------------------------------------------------------
 */
#define CNS_TAG_ROOT "root"
    #define CNS_TAG_DEF_PARAMS              "defaultparameters"
        #define CNS_TAG_ATTR_SIZE           "size"
        #define CNS_TAG_ATTR_MOVESPEED      "movespeed"
        #define CNS_TAG_ATTR_ROTATIONSPEED  "rotationspeed"
    #define CNS_TAG_AGENTS                  "agents"
    #define CNS_TAG_AGENT                   "agent"
    #define CNS_TAG_MAP                     "map"
    #define CNS_TAG_GRID                    "grid"
        #define CNS_TAG_ROW                 "row"
    #define CNS_TAG_ALGORITHM               "algorithm"
    #define CNS_TAG_WEIGHT                  "weight"
    #define CNS_TAG_HWEIGHT                 "hweight"
    #define CNS_TAG_CONNECTEDNESS           "connectedness"
    #define CNS_TAG_ALLOW_AA                "allowanyangle"
    #define CNS_TAG_PRIORITIZATION          "prioritization"
    #define CNS_TAG_RESCHEDULING            "rescheduling"
    #define CNS_TAG_STARTSAFEINTERVAL       "startsafeinterval"
    #define CNS_TAG_TIMELIMIT               "timelimit"
    #define CNS_TAG_PLANFORTURNS            "planforturns"
    #define CNS_TAG_ADDITIONALWAIT          "waitbeforemove"
    #define CNS_TAG_INFLATEINTERVALS        "inflatecollisionintervals"
    #define CNS_TAG_OPTIONS                 "options"
    #define CNS_TAG_LOGLVL                  "loglevel"
    #define CNS_TAG_LOGPATH                 "logpath"
    #define CNS_TAG_LOGFILENAME             "logfilename"
    #define CNS_TAG_DYNAMICOBSTACLES        "dynamicobstacles"
    #define CNS_TAG_OBSTACLE                "obstacle"
    #define CNS_TAG_LOG                     "log"
        #define CNS_TAG_MAPFN               "mapfilename"
        #define CNS_TAG_TASKFN              "taskfilename"
        #define CNS_TAG_CONFIGFN            "configfilename"
        #define CNS_TAG_OBSFN               "obstaclesfilename"
        #define CNS_TAG_SUM                 "summary"
        #define CNS_TAG_PATH                "path"
        #define CNS_TAG_ROW                 "row"
        #define CNS_TAG_AGENT               "agent"
        #define CNS_TAG_LOWLEVEL            "lowlevel"
            #define CNS_TAG_SECTION         "section"
            #define CNS_TAG_STEP            "step"
            #define CNS_TAG_OPEN            "open"
            #define CNS_TAG_NODE            "node"
            #define CNS_TAG_CLOSE           "close"

/*
 * End of XML files tags -------------------------------------------------------
 */

/*
 * XML files tag's attributes --------------------------------------------------
 */
    #define CNS_TAG_ATTR_HEIGHT         "height"
    #define CNS_TAG_ATTR_WIDTH          "width"
    #define CNS_TAG_ATTR_NUMOFSTEPS     "numberofsteps"
    #define CNS_TAG_ATTR_TOTALNODES     "totalnodescreated"
    #define CNS_TAG_ATTR_NODES          "nodescreated"
    #define CNS_TAG_ATTR_DURATION       "duration"
    #define CNS_TAG_ATTR_PATHLENGTH     "pathlength"
    #define CNS_TAG_ATTR_RUNTIME        "runtime"
    #define CNS_TAG_ATTR_X              "x"
    #define CNS_TAG_ATTR_Y              "y"
    #define CNS_TAG_ATTR_NUM            "number"
    #define CNS_TAG_ATTR_ID             "id"
    #define CNS_TAG_ATTR_SIZE           "size"
    #define CNS_TAG_ATTR_RSPEED         "rotationspeed"
    #define CNS_TAG_ATTR_MSPEED         "movespeed"
    #define CNS_TAG_ATTR_SHEADING       "start.heading"
    #define CNS_TAG_ATTR_GHEADING       "goal.heading"
    #define CNS_TAG_ATTR_F              "F"
    #define CNS_TAG_ATTR_G              "g"
    #define CNS_TAG_ATTR_PARX           "parent_x"
    #define CNS_TAG_ATTR_PARY           "parent_y"
    #define CNS_TAG_ATTR_VALUE          "value"
    #define CNS_TAG_ATTR_SX             "start.x"
    #define CNS_TAG_ATTR_SY             "start.y"
    #define CNS_TAG_ATTR_SH             "start.heading"
    #define CNS_TAG_ATTR_GX             "goal.x"
    #define CNS_TAG_ATTR_GY             "goal.y"
    #define CNS_TAG_ATTR_GH             "goal.heading"
    #define CNS_TAG_ATTR_PF             "pathfound"
    #define CNS_TAG_ATTR_SOLVED         "solved"
        #define CNS_TAG_ATTR_TRUE       "true"
        #define CNS_TAG_ATTR_FALSE      "false"
    #define CNS_TAG_ATTR_TRIES          "tries"
    #define CNS_TAG_ATTR_AGENTSSOLVED   "agentssolved"
    #define CNS_TAG_ATTR_PATHSFOUND     "pathsfound"
    #define CNS_TAG_ATTR_PATHFOUND      "pathfound"
    #define CNS_TAG_ATTR_MAXNODESCR     "maxnodescreated"
    #define CNS_TAG_ATTR_FLOWTIME       "flowtime"
    #define CNS_TAG_ATTR_AVGLENGTH      "avglength"
    #define CNS_TAG_ATTR_MAKESPAN       "makespan"

/*
 * End of XML files tag's attributes -------------------------------------------
 */

#define CN_NO_DIR        0
#define CN_LEFT_DIR      3
#define CN_UP_DIR        4
#define CN_RIGHT_DIR     5
#define CN_DOWN_DIR      6
#define CN_GOAL_DIR      9

#define CN_PARALLEL          1
#define CN_COINCIDENT        2
#define CN_NONINTERSECTING   3
#define CN_INTERSECTING      4



#endif
