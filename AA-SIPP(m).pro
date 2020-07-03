TARGET = AAt-SIPP-m
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x
INCLUDEPATH += C:/Users/Dell/Documents/boost_1_72_0
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
    aa_sipp.cpp \
    constraints.cpp \
    task.cpp \
    dynamicobstacles.cpp

HEADERS += \
    primitive.h \
    tinyxml2.h \
    searchresult.h \
    gl_const.h \
    xmlLogger.h \
    mission.h \
    map.h \
    config.h \
    aa_sipp.h \
    structs.h \
    constraints.h \
    lineofsight.h \
    task.h \
    dynamicobstacles.h
