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
    isCalculating = false;
    defaultConsoleColor = new QColor("white");
    errorConsoleColor = new QColor("red");
    ui->consoleOutput->setTextColor(*defaultConsoleColor);


    connect(ui->addPointButton, SIGNAL(released()), this, SLOT(addPoint()));

    connect(ui->calculateButton, SIGNAL(released()), this, SLOT(processInverseKinematicRequest()));

    connect(ui->launchButton, SIGNAL(released()), this, SLOT(launch()));
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
        updateWidgetsEnableLaunch();
    }
    else
    {
        isLaunched=true;
        updateWidgetsEnableLaunch();
        process->start("bash -c \"source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://"+ui->ipLineEdit->text()+" && export ROS_HOSTNAME=`hostname --ip-address` && rosrun object_displacer object_displacer"+/*Adding here the code for the points*/"\"");
        //process->start("bash -c \"source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://"+ui->ipLineEdit->text()+" && export ROS_HOSTNAME=`hostname --ip-address` && echo $ROS_HOSTNAME && echo $ROS_MASTER_URI\"");
    }
}

void MainWindow::addPoint()
{
    const int currentRow = ui->pointList->rowCount();
    ui->pointList->setRowCount(currentRow + 1);
    ui->pointList->setItem(currentRow, 0, new QTableWidgetItem(ui->xSpinBox->text()));
    ui->pointList->setItem(currentRow, 1, new QTableWidgetItem(ui->ySpinBox->text()));
    ui->pointList->setItem(currentRow, 2, new QTableWidgetItem(ui->zSpinBox->text()));
    ui->pointList->setItem(currentRow, 3, new QTableWidgetItem(ui->thetaSpinBox->text()));
    ui->pointList->setItem(currentRow, 4, new QTableWidgetItem(ui->phiSpinBox->text()));
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
    updateWidgetsEnableLaunch();
}

void MainWindow::updateWidgetsEnableLaunch()
{
    ui->addPointButton->setEnabled(!isLaunched);
    ui->xSpinBox->setEnabled(!isLaunched);
    ui->ySpinBox->setEnabled(!isLaunched);
    ui->zSpinBox->setEnabled(!isLaunched);
    ui->thetaSpinBox->setEnabled(!isLaunched);
    ui->phiSpinBox->setEnabled(!isLaunched);
    ui->xLabel->setEnabled(!isLaunched);
    ui->yLabel->setEnabled(!isLaunched);
    ui->zLabel->setEnabled(!isLaunched);
    ui->thetaLabel->setEnabled(!isLaunched);
    ui->phiLabel->setEnabled(!isLaunched);
    ui->ipLineEdit->setEnabled(!isLaunched);
    ui->iplabel->setEnabled(!isLaunched);
    ui->inverseKinGroupBox->setEnabled(!isLaunched);
    if (isLaunched)
    {
        ui->launchButton->setText("Terminate");
        ui->pointList->setEditTriggers(QAbstractItemView::NoEditTriggers);
    }
    else
    {
        ui->launchButton->setText("Launch");
        ui->pointList->setEditTriggers(QAbstractItemView::DoubleClicked);
    }
}

void MainWindow::processInverseKinematicRequest()
{
    if (isCalculating)
    {
        // Abort calculations
        // isCalculating = false;
        // updateWidgetsEnableCalculate();
    }
    else
    {
        isCalculating = true;
        updateWidgetsEnableCalculate();
        calculateInverseKinematic();
    }
}

void MainWindow::updateWidgetsEnableCalculate()
{
    ui->addPointButton->setEnabled(!isCalculating);
    ui->xSpinBox->setEnabled(!isCalculating);
    ui->ySpinBox->setEnabled(!isCalculating);
    ui->zSpinBox->setEnabled(!isCalculating);
    ui->thetaSpinBox->setEnabled(!isCalculating);
    ui->phiSpinBox->setEnabled(!isCalculating);
    ui->xLabel->setEnabled(!isCalculating);
    ui->yLabel->setEnabled(!isCalculating);
    ui->zLabel->setEnabled(!isCalculating);
    ui->thetaLabel->setEnabled(!isCalculating);
    ui->phiLabel->setEnabled(!isCalculating);
    ui->ipLineEdit->setEnabled(!isCalculating);
    ui->iplabel->setEnabled(!isCalculating);
    ui->launchButton->setEnabled(!isCalculating);
    if (isCalculating)
    {
        ui->calculateButton->setText("Calculating");
        ui->pointList->setEditTriggers(QAbstractItemView::NoEditTriggers);
    }
    else
    {
        ui->calculateButton->setText("Calculate");
        ui->pointList->setEditTriggers(QAbstractItemView::DoubleClicked);
    }
}

void MainWindow::calculateInverseKinematic()
{
    vector< vector<double> > outputAngles;
    outputAngles.resize(ui->pointList->rowCount(), vector<double>(5, 0));
    double x3, z3, c3, s3;
    ofstream f;
    f.open("results");
    for (int i=0; i<ui->pointList->rowCount(); i++)
    {
        // theta 1
        outputAngles[i][0] = atan2(getPointValue(i, 1), getPointValue(i, 0))-degToRad(THETA1_OFFSET);

        x3 = sqrt(pow(getPointValue(i, 0), 2) + pow(getPointValue(i, 1), 2)) - L4*cos(getPointValue(i, 3)) - OFFSET;
        z3 = getPointValue(i, 2) - L4*sin(getPointValue(i, 3)) - L1;

        // theta 3
        c3 = (pow(x3, 2) + pow(z3, 2) - pow(L3, 2) - pow(L2, 2))/(2*L3*L2);
        s3 = ELBOW_DIR*sqrt(1-pow(c3, 2));

        outputAngles[i][2] = atan2(s3, c3) + PI/2 - degToRad(THETA3_OFFSET);

        // theta 2
        outputAngles[i][1] = atan2(z3, x3) - atan2(L3*s3, L2+L3*c3) - PI/2 - degToRad(THETA2_OFFSET);

        // theta 4
        outputAngles[i][3] = getPointValue(i, 3) - outputAngles[i][1] - outputAngles[i][2] - PI/2 - degToRad(THETA4_OFFSET);

        // theta 5
        outputAngles[i][4] = getPointValue(i, 4);

        ui->consoleOutput->append(QString::number(outputAngles[i][0])+" "+QString::number(outputAngles[i][1])+" "+QString::number(outputAngles[i][2])+" "+QString::number(outputAngles[i][3])+" "+QString::number(outputAngles[i][4])+" ");
        // copy to file
        for (int j=0; j<4; j++)
        {
            f << outputAngles[i][j] << " ";
        }
        f << outputAngles[i][4] << endl;
    }
    f.close();
    // reset the window when finished
    isCalculating = false;
    updateWidgetsEnableCalculate();
}

double MainWindow::getPointValue(int point, int coordinate)
{
    return ui->pointList->item(point, coordinate)->text().toDouble();
}

double MainWindow::degToRad(double degreeAngle)
{
    return (PI*degreeAngle)/180.0;
}

MainWindow::~MainWindow()
{
    delete ui;
}
