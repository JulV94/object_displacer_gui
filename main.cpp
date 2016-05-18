#include "mainwindow.h"
#include <QApplication>
#include <GL/glut.h>

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    QApplication a(argc, argv);
    a.setStyleSheet("file:///"+QApplication::applicationDirPath()+"/stylesheet.css");
    MainWindow w;
    w.show();

    return a.exec();
}
