#include "mainwindow.h"
#include <QApplication>
#include <GL/glut.h>

int main(int argc, char *argv[])
{
    // Initialize glut for simulation in OpenGL
    glutInit(&argc, argv);
    // Create a Qt application
    QApplication a(argc, argv);
    // Bind a css stylesheet
    a.setStyleSheet("file:///"+QApplication::applicationDirPath()+"/stylesheet.css");
    // Create a window object
    MainWindow w;
    // Display it
    w.show();

    return a.exec();
}
