#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyleSheet("file:///"+QApplication::applicationDirPath()+"/stylesheet.css");
    MainWindow w;
    w.show();

    return a.exec();
}
