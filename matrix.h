/*
Author : Julien Van Loo
Date : 20 april 2013
Class that provides tools for matrices
*/

#ifndef MATRIX_H_INCLUDED
#define MATRIX_H_INCLUDED

#include <iostream>
#include <vector>
#include <fstream>
//#include <cmath>

using namespace std;

class Matrix
{
    public:
        Matrix(unsigned int size);  // OK
        Matrix(unsigned int line, unsigned int column);  // OK
        Matrix(const Matrix &srcMatrix);  // OK
        Matrix(string filename);  // OK
        void setValue(double value, unsigned int line, unsigned int column);  // OK
        double value(unsigned int line, unsigned int column) const;  // OK
        void setLine(double line[], unsigned int lineNbr, unsigned int arraySize);  // OK
        void setLine(const vector<double> &line, unsigned int lineNbr);  // OK
        void setMatrix(const vector<double> &matrix);  // OK
        void setIdentity();  // OK
        vector<double> returnMatrix() const;  // OK
        unsigned int returnNbrLines() const;  // OK
        unsigned int returnNbrColumns() const;  // OK
        bool isSquareMat() const;  // OK
        void printMatrix() const;  // OK
        Matrix& operator=(const Matrix &srcMatrix);  // OK
        Matrix& operator+=(const Matrix &matrix2);  // OK
        Matrix& operator-=(const Matrix &matrix2);  // OK
        Matrix& operator*=(const double &cst);  // OK
        Matrix& operator/=(const double &cst);  // OK
        Matrix& operator*=(const Matrix &matrix2);  // OK
        Matrix pow(int exp);  // OK
        Matrix systemSolve(const Matrix &indepMat);
        void transpose();  // OK
        void diagonalise();  // OK
        void triangulate();  // OK
        double trace() const;  // OK
        double diagonalMult() const;  // OK
        double determinant() const;  // OK
        void invert();  // OK
        void save(string filename) const;  // OK
        void load(string filename);  // OK
        ~Matrix();  // OK

    private:
        vector<double> grid;
        unsigned int lines;
        unsigned int columns;
        bool squareMatrix;
};

//Opérateurs surchargés
Matrix operator+(const Matrix &matrix1, const Matrix &matrix2);  // OK
Matrix operator-(const Matrix &matrix1, const Matrix &matrix2);  // OK
Matrix operator*(const double &cst, const Matrix &matrix);  // OK
Matrix operator*(const Matrix &matrix, const double &cst);  // OK
Matrix operator/(const Matrix &matrix, const double &cst);  // OK
Matrix operator*(const Matrix &matrix1, const Matrix &matrix2);  // OK

#endif // MATRIX_H_INCLUDED
