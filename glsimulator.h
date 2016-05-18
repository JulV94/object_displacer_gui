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
    void launchSimulation(QString filename);

private:
    void calculateDirectKinematic();
    vector< vector<double> > invKin;
    vector<Matrix> points;
    QTimer timer;
    unsigned int counter;
    bool isSimulating;
};

#endif // GLSIMULATOR_H
