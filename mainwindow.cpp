#include "mainwindow.h"
#include "ui_mainwindow.h"

// Initialize the window
MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWindow)
{
    // Initialize components and variables
    ui->setupUi(this);
    process = new QProcess();
    isLaunched = false;
    isCalculating = false;
    defaultConsoleColor = new QColor("white");
    errorConsoleColor = new QColor("red");
    ui->consoleOutput->setTextColor(*defaultConsoleColor);
    resultFilePath = new QString(QApplication::applicationDirPath()+"/results");

    // Connect components with each other
    connect(ui->addPointButton, SIGNAL(released()), this, SLOT(addPoint()));
    connect(ui->exportPointsButton, SIGNAL(released()), this, SLOT(exportPoints()));
    connect(ui->importPointsButton, SIGNAL(released()), this, SLOT(importPoints()));

    connect(ui->calculateButton, SIGNAL(released()), this, SLOT(processInverseKinematicRequest()));

    connect(ui->launchButton, SIGNAL(released()), this, SLOT(launch()));
    connect(process, SIGNAL(readyReadStandardError()), this, SLOT(updateOutputError()));
    connect(process, SIGNAL(readyReadStandardOutput()), this, SLOT(updateOutputStd()));
    connect(process, SIGNAL(started()), this, SLOT(processStarted()));
    connect(process, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(processFinished(int, QProcess::ExitStatus)));

    connect(ui->simulateButton, SIGNAL(released()), this, SLOT(launchSimulation()));
}

// Launch the ros program
void MainWindow::launch()
{
    // If the program is running, terminate it
    if (isLaunched)
    {
        process->terminate();
        isLaunched=false;
        updateWidgetsEnableLaunch();
    }
    // Else launch it
    else
    {
        isLaunched=true;
        updateWidgetsEnableLaunch();
        process->start("bash -c \"source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://"+ui->ipLineEdit->text()+" && export ROS_HOSTNAME=`hostname -I` && rosrun object_displacer object_displacer "+*resultFilePath+"\"");
    }
}

// Add a point in the tableWidget component
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

// Export points of the tableWidget to a file
void MainWindow::exportPoints()
{
    QString filename;
    filename = QFileDialog::getSaveFileName(this, tr("Export Points"), NULL, NULL);
    ofstream f;
    f.open(filename.toStdString().c_str());
    for (int i=0; i<ui->pointList->rowCount(); i++)
    {
        for (int j=0; j<4; j++)
        {
            f << getPointValue(i, j) << " ";
        }
        if (i < ui->pointList->rowCount()-1)
        {
            f << getPointValue(i, 4) << endl;
        }
        else
        {
            f << getPointValue(i, 4);
        }
    }
    f.close();
}

// Import points form file to tableWidget
void MainWindow::importPoints()
{
    QString filename;
    string value;
    int row=0;
    filename = QFileDialog::getOpenFileName(this, tr("Import Points"), NULL, NULL);
    ifstream f;
    f.open(filename.toStdString().c_str());
    if (f.is_open())
    {
        ui->pointList->clearContents();
        while (!f.eof())
        {
            ui->pointList->setRowCount(row + 1);
            for (int i=0; i<5; i++)
            {
                f >> value;
                ui->pointList->setItem(row, i, new QTableWidgetItem(QString::fromStdString(value)));
            }
            row++;
        }
        f.close();
    }
}

// Display errors incoming from ros program into the console component od the window
void MainWindow::updateOutputError()
{
    ui->consoleOutput->setTextColor(*errorConsoleColor);
    ui->consoleOutput->append(QString(process->readAllStandardError()));
    ui->consoleOutput->setTextColor(*defaultConsoleColor);
}

// Display standard messages incoming from ros program into the console component of the window
void MainWindow::updateOutputStd()
{
    ui->consoleOutput->append(QString(process->readAllStandardOutput()));
}

// Trigger from the start of the ros program
void MainWindow::processStarted()
{
    ui->consoleOutput->append("Command launched");
}

// Trigger of the end of the ros program (display status of the execution)
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

// Enable or disable components when ros program is running
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

// Send a request to calculate the inverse kinematic of the given points (points of tableWidget)
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

// Enable or disable components when calculating inverse kinematic
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

// Calculate the inverse kinematic
void MainWindow::calculateInverseKinematic()
{
    ui->calculateLabel->setText("Calculating...");
    vector< vector<double> > outputAngles;
    outputAngles.resize(ui->pointList->rowCount(), vector<double>(5, 0));
    double x3, z3, c3, s3;
    ofstream f;
    f.open(resultFilePath->toStdString().c_str());
    // For each position, calculate the angles required by each joint to achieve the given position
    for (int i=0; i<ui->pointList->rowCount(); i++)
    {
        // theta 1
        outputAngles[i][0] = -atan2(getPointValue(i, 1), getPointValue(i, 0))+degToRad(THETA1_OFFSET) - PI;

        x3 = sqrt(pow(getPointValue(i, 0), 2) + pow(getPointValue(i, 1), 2)) - L4*cos(getPointValue(i, 3)) - OFFSET;
        z3 = getPointValue(i, 2) - L4*sin(getPointValue(i, 3)) - L1;

        // theta 3
        c3 = (pow(x3, 2) + pow(z3, 2) - pow(L3, 2) - pow(L2, 2))/(2*L3*L2);
        s3 = ELBOW_DIR*sqrt(1-pow(c3, 2));

        outputAngles[i][2] = atan2(s3, c3) + PI/2 - degToRad(THETA3_OFFSET);

        // theta 2
        outputAngles[i][1] = atan2(z3, x3) - atan2(L3*s3, L2+L3*c3) - PI/2 - degToRad(THETA2_OFFSET) + 0.188;

        // theta 4
        outputAngles[i][3] = getPointValue(i, 3) - outputAngles[i][1] - outputAngles[i][2] - PI/2 - degToRad(THETA4_OFFSET);

        // theta 5
        outputAngles[i][4] = getPointValue(i, 4);

        for (int j=0; j<5; j++)
        {
            outputAngles[i][j]= -outputAngles[i][j];
        }

        // Show result in console component
        ui->consoleOutput->append(QString::number(outputAngles[i][0])+" "+QString::number(outputAngles[i][1])+" "+QString::number(outputAngles[i][2])+" "+QString::number(outputAngles[i][3])+" "+QString::number(outputAngles[i][4])+" ");
        // copy the result to file
        for (int j=0; j<4; j++)
        {
            f << outputAngles[i][j] << " ";
        }
        if (i < ui->pointList->rowCount()-1)
        {
            f << outputAngles[i][4] << endl;
        }
        else
        {
            f << outputAngles[i][4];
        }
    }
    f.close();
    ui->calculateLabel->setText("Angles generated");
    // reset the window when finished
    isCalculating = false;
    updateWidgetsEnableCalculate();
}

// Get value of a cell in the tableWidget
double MainWindow::getPointValue(int point, int coordinate)
{
    return ui->pointList->item(point, coordinate)->text().toDouble();
}

// Convert degree to radiant
double MainWindow::degToRad(double degreeAngle)
{
    return (PI*degreeAngle)/180.0;
}

// Request a simulation
void MainWindow::launchSimulation()
{
    ui->simWidget->launchSimulation(*resultFilePath);
}

// Destructor of the object
MainWindow::~MainWindow()
{
    delete ui;
}
