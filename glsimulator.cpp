#include "glsimulator.h"

GLSimulator::GLSimulator(QWidget *parent) :
    QGLWidget(parent)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(updateGL()));
    counter = 0;
    isSimulating = false;
}

void GLSimulator::initializeGL()
{
    glClearColor(0,0,0,1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
}

void GLSimulator::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ground
    glColor3f(0.5, 0.5, 0.5);
    glBegin(GL_TRIANGLES);
    glVertex3f(-SIM_WALL_SIZE, SIM_WALL_SIZE, 0.0);
    glVertex3f(-SIM_WALL_SIZE,-SIM_WALL_SIZE,0.0);
    glVertex3f(SIM_WALL_SIZE,-SIM_WALL_SIZE,0.0);
    glEnd();
    glBegin(GL_TRIANGLES);
    glVertex3f(-SIM_WALL_SIZE, SIM_WALL_SIZE, 0.0);
    glVertex3f(SIM_WALL_SIZE,SIM_WALL_SIZE,0.0);
    glVertex3f(SIM_WALL_SIZE,-SIM_WALL_SIZE,0.0);
    glEnd();

    if (isSimulating && counter < invKin.size())
    {

        glLineWidth(2.5);
        glColor3f(0.0, 1.0, 1.0);
        glBegin(GL_LINES);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(0,0), SIM_ZOOM_FACTOR*points[counter].value(0,1), SIM_ZOOM_FACTOR*points[counter].value(0,2));
        glEnd();

        glColor3f(1.0, 0.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(0,0), SIM_ZOOM_FACTOR*points[counter].value(0,1), SIM_ZOOM_FACTOR*points[counter].value(0,2));
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(1,0), SIM_ZOOM_FACTOR*points[counter].value(1,1), SIM_ZOOM_FACTOR*points[counter].value(1,2));
        glEnd();

        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(1,0), SIM_ZOOM_FACTOR*points[counter].value(1,1), SIM_ZOOM_FACTOR*points[counter].value(1,2));
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(2,0), SIM_ZOOM_FACTOR*points[counter].value(2,1), SIM_ZOOM_FACTOR*points[counter].value(2,2));
        glEnd();

        glColor3f(0.0, 0.0, 1.0);
        glBegin(GL_LINES);
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(2,0), SIM_ZOOM_FACTOR*points[counter].value(2,1), SIM_ZOOM_FACTOR*points[counter].value(2,2));
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(3,0), SIM_ZOOM_FACTOR*points[counter].value(3,1), SIM_ZOOM_FACTOR*points[counter].value(3,2));
        glEnd();

        glColor3f(1.0, 1.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(3,0), SIM_ZOOM_FACTOR*points[counter].value(3,1), SIM_ZOOM_FACTOR*points[counter].value(3,2));
        glVertex3f(SIM_ZOOM_FACTOR*points[counter].value(4,0), SIM_ZOOM_FACTOR*points[counter].value(4,1), SIM_ZOOM_FACTOR*points[counter].value(4,2));
        glEnd();

        counter++;
    }
    else
    {
        timer.stop();
        counter = 0;
        isSimulating = false;
    }
}

void GLSimulator::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (float)w/h, 0.01, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(-3,-5,2.5, 0,0,0, 0,0,1);
}

void GLSimulator::launchSimulation(QString filename)
{
    string value;
    ifstream f;
    f.open(filename.toStdString().c_str());
    while (!f.eof())
    {
        vector<double> currentPoint;
        for (int i=0; i<5; i++)
        {
            f >> value;
            currentPoint.push_back(atof(value.c_str()));
        }
        invKin.push_back(currentPoint);
    }
    f.close();
    calculateDirectKinematic();
    isSimulating = true;
    timer.start(1000);
}

void GLSimulator::calculateDirectKinematic()
{
    Matrix T(4);
    Matrix result(5,3);
    Matrix T1(4);
    Matrix T2(4);
    Matrix T3(4);
    Matrix T4(4);
    Matrix T5(4);

    for (unsigned int i=0; i<invKin.size(); i++)
    {
        T1.setValue(cos(invKin[i][0]),0,0);
        T1.setValue(-sin(invKin[i][0])*cos(PI/2),0,1);
        T1.setValue(sin(invKin[i][0])*sin(PI/2),0,2);
        T1.setValue(OFFSET*cos(invKin[i][0]),0,3);
        T1.setValue(sin(invKin[i][0]),1,0);
        T1.setValue(cos(invKin[i][0])*cos(PI/2),1,1);
        T1.setValue(-cos(invKin[i][0])*sin(PI/2),1,2);
        T1.setValue(OFFSET*sin(invKin[i][0]),1,3);
        T1.setValue(0,2,0);
        T1.setValue(sin(PI/2),2,1);
        T1.setValue(cos(PI/2),2,2);
        T1.setValue(L1,2,3);
        T1.setValue(0,3,0);
        T1.setValue(0,3,1);
        T1.setValue(0,3,2);
        T1.setValue(1,3,3);
        T2.setValue(cos(invKin[i][1]),0,0);
        T2.setValue(-sin(invKin[i][1])*cos(0),0,1);
        T2.setValue(sin(invKin[i][1])*sin(0),0,2);
        T2.setValue(L2*cos(invKin[i][1]),0,3);
        T2.setValue(sin(invKin[i][1]),1,0);
        T2.setValue(cos(invKin[i][1])*cos(0),1,1);
        T2.setValue(-cos(invKin[i][1])*sin(0),1,2);
        T2.setValue(L2*sin(invKin[i][1]),1,3);
        T2.setValue(0,2,0);
        T2.setValue(sin(0),2,1);
        T2.setValue(cos(0),2,2);
        T2.setValue(0,2,3);
        T2.setValue(0,3,0);
        T2.setValue(0,3,1);
        T2.setValue(0,3,2);
        T2.setValue(1,3,3);
        T3.setValue(cos(invKin[i][2]),0,0);
        T3.setValue(-sin(invKin[i][2])*cos(0),0,1);
        T3.setValue(sin(invKin[i][2])*sin(0),0,2);
        T3.setValue(L3*cos(invKin[i][2]),0,3);
        T3.setValue(sin(invKin[i][2]),1,0);
        T3.setValue(cos(invKin[i][2])*cos(0),1,1);
        T3.setValue(-cos(invKin[i][2])*sin(0),1,2);
        T3.setValue(L3*sin(invKin[i][2]),1,3);
        T3.setValue(0,2,0);
        T3.setValue(sin(0),2,1);
        T3.setValue(cos(0),2,2);
        T3.setValue(0,2,3);
        T3.setValue(0,3,0);
        T3.setValue(0,3,1);
        T3.setValue(0,3,2);
        T3.setValue(1,3,3);
        T4.setValue(cos(invKin[i][3]),0,0);
        T4.setValue(-sin(invKin[i][3])*cos(-PI/2),0,1);
        T4.setValue(sin(invKin[i][3])*sin(-PI/2),0,2);
        T4.setValue(0*cos(invKin[i][3]),0,3);
        T4.setValue(sin(invKin[i][3]),1,0);
        T4.setValue(cos(invKin[i][3])*cos(-PI/2),1,1);
        T4.setValue(-cos(invKin[i][3])*sin(-PI/2),1,2);
        T4.setValue(0*sin(invKin[i][3]),1,3);
        T4.setValue(0,2,0);
        T4.setValue(sin(-PI/2),2,1);
        T4.setValue(cos(-PI/2),2,2);
        T4.setValue(0,2,3);
        T4.setValue(0,3,0);
        T4.setValue(0,3,1);
        T4.setValue(0,3,2);
        T4.setValue(1,3,3);
        T5.setValue(cos(invKin[i][4]),0,0);
        T5.setValue(-sin(invKin[i][4])*cos(0),0,1);
        T5.setValue(sin(invKin[i][4])*sin(0),0,2);
        T5.setValue(0*cos(invKin[i][4]),0,3);
        T5.setValue(sin(invKin[i][4]),1,0);
        T5.setValue(cos(invKin[i][4])*cos(0),1,1);
        T5.setValue(-cos(invKin[i][4])*sin(0),1,2);
        T5.setValue(0*sin(invKin[i][4]),1,3);
        T5.setValue(0,2,0);
        T5.setValue(sin(0),2,1);
        T5.setValue(cos(0),2,2);
        T5.setValue(L4,2,3);
        T5.setValue(0,3,0);
        T5.setValue(0,3,1);
        T5.setValue(0,3,2);
        T5.setValue(1,3,3);

        T=T1;
        result.setValue(T.value(0, 3),0,0);
        result.setValue(T.value(1, 3),0,1);
        result.setValue(T.value(2, 3),0,2);
        T*=T2;
        result.setValue(T.value(0, 3),1,0);
        result.setValue(T.value(1, 3),1,1);
        result.setValue(T.value(2, 3),1,2);
        T*=T3;
        result.setValue(T.value(0, 3),2,0);
        result.setValue(T.value(1, 3),2,1);
        result.setValue(T.value(2, 3),2,2);
        T*=T4;
        result.setValue(T.value(0, 3),3,0);
        result.setValue(T.value(1, 3),3,1);
        result.setValue(T.value(2, 3),3,2);
        T*=T5;
        result.setValue(T.value(0, 3),4,0);
        result.setValue(T.value(1, 3),4,1);
        result.setValue(T.value(2, 3),4,2);

        points.push_back(result);
    }
}
