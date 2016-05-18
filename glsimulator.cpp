#include "glsimulator.h"

GLSimulator::GLSimulator(QWidget *parent) :
    QGLWidget(parent)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(updateGL()));
    counter = 0;
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

    if (counter < pointList.size())
    {
        glLineWidth(2.5);
        glColor3f(1.0, 0.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(15, 0, 0);
        glEnd();

        glLineWidth(2.5);
        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(15, 5, 0);
        glEnd();

        glLineWidth(2.5);
        glColor3f(0.0, 0.0, 1.0);
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(15, 10, 0);
        glEnd();

        glLineWidth(2.5);
        glColor3f(1.0, 1.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(pointList[counter][0], pointList[counter][1], pointList[counter][2]);
        glEnd();

        counter++;
    }
    else
    {
        timer.stop();
        counter = 0;
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
    gluLookAt(0,0,5, 0,0,0, 0,1,0);
}

void GLSimulator::launchSimulation(QTableWidget *tableWidget)
{
    pointList.resize(tableWidget->rowCount(), vector<double>(5, 0));

    for (int i=0; i<tableWidget->rowCount(); i++)
    {
        for (int j=0; j<5; j++)
        {
            pointList[i][j] = tableWidget->item(i, j)->text().toDouble();
        }
    }
    calculateDirectKinematic();
    timer.start(1000);
}

void GLSimulator::calculateDirectKinematic()
{
    for (unsigned int i=0; i<pointList.size(); i++)
    {
        Matrix T1(4);
        T1.setValue(cos(Q1),0,0);
        T1.setValue(-sin(Q1)*cos(PI/2),0,1);
        T1.setValue(sin(Q1)*sin(PI/2),0,2);
        T1.setValue(OFFSET*cos(Q1),0,3);
        T1.setValue(sin(Q1),1,0);
        T1.setValue(cos(Q1)*cos(PI/2),1,1);
        T1.setValue(-cos(Q1)*sin(PI/2),1,2);
        T1.setValue(OFFSET*sin(Q1),1,3);
        T1.setValue(0,2,0);
        T1.setValue(sin(PI/2),2,1);
        T1.setValue(cos(PI/2),2,2);
        T1.setValue(L1,2,3);
        T1.setValue(0,3,0);
        T1.setValue(0,3,1);
        T1.setValue(0,3,2);
        T1.setValue(1,3,3);
        Matrix T2(4);
        T2.setValue(cos(Q2),0,0);
        T2.setValue(-sin(Q2)*cos(0),0,1);
        T2.setValue(sin(Q2)*sin(0),0,2);
        T2.setValue(L2*cos(Q2),0,3);
        T2.setValue(sin(Q2),1,0);
        T2.setValue(cos(Q2)*cos(0),1,1);
        T2.setValue(-cos(Q2)*sin(0),1,2);
        T2.setValue(L2*sin(Q2),1,3);
        T2.setValue(0,2,0);
        T2.setValue(sin(0),2,1);
        T2.setValue(cos(0),2,2);
        T2.setValue(0,2,3);
        T2.setValue(0,3,0);
        T2.setValue(0,3,1);
        T2.setValue(0,3,2);
        T2.setValue(1,3,3);
        Matrix T3(4);
        T3.setValue(cos(Q3),0,0);
        T3.setValue(-sin(Q3)*cos(0),0,1);
        T3.setValue(sin(Q3)*sin(0),0,2);
        T3.setValue(L3*cos(Q3),0,3);
        T3.setValue(sin(Q3),1,0);
        T3.setValue(cos(Q3)*cos(0),1,1);
        T3.setValue(-cos(Q3)*sin(0),1,2);
        T3.setValue(L3*sin(Q3),1,3);
        T3.setValue(0,2,0);
        T3.setValue(sin(0),2,1);
        T3.setValue(cos(0),2,2);
        T3.setValue(0,2,3);
        T3.setValue(0,3,0);
        T3.setValue(0,3,1);
        T3.setValue(0,3,2);
        T3.setValue(1,3,3);
        Matrix T4(4);
        T4.setValue(cos(Q4),0,0);
        T4.setValue(-sin(Q4)*cos(-PI/2),0,1);
        T4.setValue(sin(Q4)*sin(-PI/2),0,2);
        T4.setValue(0*cos(Q4),0,3);
        T4.setValue(sin(Q4),1,0);
        T4.setValue(cos(Q4)*cos(-PI/2),1,1);
        T4.setValue(-cos(Q4)*sin(-PI/2),1,2);
        T4.setValue(0*sin(Q4),1,3);
        T4.setValue(0,2,0);
        T4.setValue(sin(-PI/2),2,1);
        T4.setValue(cos(-PI/2),2,2);
        T4.setValue(0,2,3);
        T4.setValue(0,3,0);
        T4.setValue(0,3,1);
        T4.setValue(0,3,2);
        T4.setValue(1,3,3);
        Matrix T5(4);
        T5.setValue(cos(Q5),0,0);
        T5.setValue(-sin(Q5)*cos(0),0,1);
        T5.setValue(sin(Q5)*sin(0),0,2);
        T5.setValue(0*cos(Q5),0,3);
        T5.setValue(sin(Q5),1,0);
        T5.setValue(cos(Q5)*cos(0),1,1);
        T5.setValue(-cos(Q5)*sin(0),1,2);
        T5.setValue(0*sin(Q5),1,3);
        T5.setValue(0,2,0);
        T5.setValue(sin(0),2,1);
        T5.setValue(cos(0),2,2);
        T5.setValue(L4,2,3);
        T5.setValue(0,3,0);
        T5.setValue(0,3,1);
        T5.setValue(0,3,2);
        T5.setValue(1,3,3);

        directKin.push_back(T1*T2*T3*T4*T5);

    }
}
