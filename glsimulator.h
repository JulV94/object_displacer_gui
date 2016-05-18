#ifndef GLSIMULATOR_H
#define GLSIMULATOR_H

#include <QGLWidget>
#include <QTableWidget>
#include <QTimer>
#include <GL/glut.h>
#include <vector>
#include <math.h>
#include "constants.h"
#include "matrix.h"

using namespace std;

class GLSimulator : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLSimulator(QWidget *parent = 0);

    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

public slots:
    void launchSimulation(QTableWidget *tableWidget);

private:
    void calculateDirectKinematic();
    vector< vector<double> > pointList;
    vector<Matrix> directKin;
    QTimer timer;
    unsigned int counter;

};

#endif // GLSIMULATOR_H
