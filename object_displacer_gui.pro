#-------------------------------------------------
#
# Project created by QtCreator 2016-05-04T13:33:11
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = object_displacer_gui
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    glsimulator.cpp \
    matrix.cpp

HEADERS  += mainwindow.h \
    constants.h \
    glsimulator.h \
    matrix.h

FORMS    += mainwindow.ui

OTHER_FILES += \
    stylesheet.css

LIBS += -lglut -lGLU
