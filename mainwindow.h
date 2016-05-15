#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QProcess>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include "constants.h"

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void launch();
    void addPoint();
    void updateOutputError();
    void updateOutputStd();
    void processStarted();
    void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void processInverseKinematicRequest();

private:
    void updateWidgetsEnableLaunch();
    void updateWidgetsEnableCalculate();
    void calculateInverseKinematic();
    double getPointValue(int point, int coordinate);
    double degToRad(double degreeAngle);
    Ui::MainWindow *ui;
    QProcess *process;
    bool isLaunched;
    bool isCalculating;
    QColor *defaultConsoleColor;
    QColor *errorConsoleColor;
    QString *resultFilePath;
};

#endif // MAINWINDOW_H
