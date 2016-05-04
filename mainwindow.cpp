#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->launchButton, SIGNAL(clicked()), this, SLOT(launch));
}

MainWindow::launch()
{
    cout << "hello";
}

MainWindow::~MainWindow()
{
    delete ui;
}
