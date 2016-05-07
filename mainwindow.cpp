#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QMessageBox"

MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    process = new QProcess();
    isLaunched = false;
    defaultConsoleColor = new QColor("white");
    errorConsoleColor = new QColor("red");
    ui->consoleOutput->setTextColor(*defaultConsoleColor);

    connect(ui->launchButton, SIGNAL(released()), this, SLOT(launch()));
    connect(ui->addPointButton, SIGNAL(released()), this, SLOT(addPoint()));

    connect(process, SIGNAL(readyReadStandardError()), this, SLOT(updateOutputError()));
    connect(process, SIGNAL(readyReadStandardOutput()), this, SLOT(updateOutputStd()));
    connect(process, SIGNAL(started()), this, SLOT(processStarted()));
    connect(process, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(processFinished(int, QProcess::ExitStatus)));
}

void MainWindow::launch()
{
    if (isLaunched)
    {
        process->terminate();
        isLaunched=false;
        updateWidgetsEnable();
    }
    else
    {
        isLaunched=true;
        updateWidgetsEnable();
        process->start("bash -c \"source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://"+ui->ipLineEdit->text()+" && export ROS_HOSTNAME=`hostname --ip-address` && rosrun object_displacer object_displacer"+/*Adding here the code for the points*/"\"");
        //process->start("bash -c \"source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://"+ui->ipLineEdit->text()+" && export ROS_HOSTNAME=`hostname --ip-address` && echo $ROS_HOSTNAME && echo $ROS_MASTER_URI\"");
    }
}

void MainWindow::addPoint()
{
    // code to add a point
    //ui->pointList->insertRow(ui->pointList->rowCount());
    const int currentRow = ui->pointList->rowCount();
    ui->pointList->setRowCount(currentRow + 1);

    ui->pointList->setItem(currentRow, 0, new QTableWidgetItem(ui->xSpinBox->value()));
    ui->pointList->setItem(currentRow, 1, new QTableWidgetItem(ui->ySpinBox->text()));
}

void MainWindow::updateOutputError()
{
    ui->consoleOutput->setTextColor(*errorConsoleColor);
    ui->consoleOutput->append(QString(process->readAllStandardError()));
    ui->consoleOutput->setTextColor(*defaultConsoleColor);
}

void MainWindow::updateOutputStd()
{
    ui->consoleOutput->append(QString(process->readAllStandardOutput()));
}

void MainWindow::processStarted()
{
    ui->consoleOutput->append("Command launched");
}

void MainWindow::processFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    if (exitStatus == QProcess::CrashExit)
    {
        // in red
        ui->consoleOutput->append("Program crashed");
    }
    else
    {
        ui->consoleOutput->append("Program finished with code "+QString::number(exitCode));
    }
    isLaunched=false;
    updateWidgetsEnable();
}

void MainWindow::updateWidgetsEnable()
{
    ui->addPointGroupBox->setEnabled(!isLaunched);
    ui->ipLineEdit->setEnabled(!isLaunched);
    ui->iplabel->setEnabled(!isLaunched);
    if (isLaunched)
    {
        ui->launchButton->setText("Terminate");
    }
    else
    {
        ui->launchButton->setText("Launch");

    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
