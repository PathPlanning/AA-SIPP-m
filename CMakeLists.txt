cmake_minimum_required( VERSION 2.8 )

project( AA-SIPP-m )

include_directories( "." )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set( SOURCE_FILES 
     main.cpp  
     tinyxml2.cpp
     xmlLogger.cpp 
     mission.cpp 
     map.cpp 
	 task.cpp
     config.cpp 
     sipp.cpp 
     aa_sipp.cpp
     constraints.cpp )

set( HEADER_FILES 
     tinyxml2.h
     searchresult.h 
     gl_const.h 
     xmlLogger.h 
     search.h 
     mission.h 
     map.h 
	 task.h
     logger.h 
     config.h 
     sipp.h 
     aa_sipp.h 
     structs.h
     constraints.h )

add_executable( AA-SIPP ${SOURCE_FILES} ${HEADER_FILES} )
