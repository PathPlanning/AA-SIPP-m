TARGET = AA-SIPP-m
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x
TEMPLATE = app
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}
SOURCES += main.cpp \
    tinyxmlparser.cpp \
    tinyxmlerror.cpp \
    tinyxml.cpp \
    tinystr.cpp \
    xmlLogger.cpp \
    mission.cpp \
    map.cpp \
    logger.cpp \
    config.cpp \
    sipp.cpp \
    aa_sipp.cpp \
    constraints.cpp

HEADERS += \
    tinyxml.h \
    tinystr.h \
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
    vector2D.h \
    constraints.h
