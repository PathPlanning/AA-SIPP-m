TARGET = AAt-SIPP-m
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x
TEMPLATE = app
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}
SOURCES += main.cpp \
    tinyxml2.cpp\
    xmlLogger.cpp \
    mission.cpp \
    map.cpp \
    config.cpp \
    sipp.cpp \
    aa_sipp.cpp \
    constraints.cpp \
    task.cpp

HEADERS += \
    tinyxml2.h \
    searchresult.h \
    gl_const.h \
    xmlLogger.h \
    search.h \
    mission.h \
    map.h \
    logger.h \
    config.h \
    sipp.h \
    aa_sipp.h \
    structs.h \
    constraints.h \
    lineofsight.h \
    task.h
