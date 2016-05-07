#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QProcess>

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

private:
    void updateWidgetsEnable();
    Ui::MainWindow *ui;
    QProcess *process;
    bool isLaunched;
    QColor *defaultConsoleColor;
    QColor *errorConsoleColor;
};

#endif // MAINWINDOW_H
