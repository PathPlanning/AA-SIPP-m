cmake_minimum_required( VERSION 2.8 )

project( AA-SIPP-m )

include_directories( "." )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set( SOURCE_FILES 
     main.cpp 
     tinyxmlparser.cpp 
     tinyxmlerror.cpp 
     tinyxml.cpp 
     tinystr.cpp 
     cXmlLogger.cpp 
     cMission.cpp 
     cMap.cpp 
     cLogger.cpp 
     cConfig.cpp 
     sipp.cpp 
     aa_sipp.cpp )

set( HEADER_FILES 
     tinyxml.h
     tinystr.h 
     searchresult.h 
     gl_const.h 
     cXmlLogger.h 
     cSearch.h 
     cMission.h 
     cMap.h 
     cLogger.h 
     cConfig.h 
     sipp.h 
     aa_sipp.h 
     structs.h )

add_executable( AA-SIPP ${SOURCE_FILES} ${HEADER_FILES} )
