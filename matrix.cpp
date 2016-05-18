/*
Author : Julien Van Loo
Date : 20 april 2013
Class that provides tools for matrices
*/

#include "matrix.h"

Matrix::Matrix(unsigned int size)
{
    grid.resize(size*size);

    lines=size;
    columns=size;
    squareMatrix=true;
}

Matrix::Matrix(unsigned int line, unsigned int column)
{
    grid.resize(line*column);
    lines=line;
    columns=column;
    squareMatrix=(line == column);
}

Matrix::Matrix(const Matrix &srcMatrix) : lines(srcMatrix.lines), columns(srcMatrix.columns), squareMatrix(srcMatrix.squareMatrix)
{
    grid.resize(lines*columns);

    grid = srcMatrix.grid;
}

Matrix::Matrix(string filename)
{
    ifstream file(filename.c_str());
    if (file.is_open())
    {
        file >> lines;
        file.seekg(1, file.cur);
        file >> columns;
        squareMatrix=(lines==columns);
        grid.resize(lines*columns);
        for (unsigned int i=0; i<lines*columns; i++)
        {
            file >> grid[i];
            file.seekg(1, file.cur);
        }
        file.close();
    }
}

void Matrix::setValue(double value, unsigned int line, unsigned int column)
{
    if (line<=lines and column<=columns)
    {
        grid[(line*columns)+column]=value;
    }
}

double Matrix::value(unsigned int line, unsigned int column) const
{
    if (line<lines and column<columns)
    {
        return grid[(line*columns)+column];
    }
    return 0.0;
}

void Matrix::setLine(double line[], unsigned int lineNbr, unsigned int arraySize)
{
    arraySize=arraySize/sizeof(double);
    if (arraySize<columns)
    {
        for (unsigned int i=0; i<arraySize; i++)
            setValue(line[i], lineNbr, i);
        for (unsigned int i=arraySize; i<columns; i++)
            setValue(0.0, lineNbr, i);
    }
    else
    {
        for (unsigned int i=0; i<columns; i++)
            setValue(line[i], lineNbr, i);
    }
}

void Matrix::setLine(const vector<double> &line, unsigned int lineNbr)
{
    if (line.size()<columns)
    {
        for (unsigned int i=0; i<line.size(); i++)
            setValue(line[i], lineNbr, i);
        for (unsigned int i=line.size(); i<columns; i++)
            setValue(0.0, lineNbr, i);
    }
    else
    {
        for (unsigned int i=0; i<columns; i++)
            setValue(line[i], lineNbr, i);
    }
}

void Matrix::setMatrix(const vector<double> &matrix)
{
    unsigned int nbrCols=matrix.size()/lines;
    if (!(matrix.size()%lines) and nbrCols == columns)
    {
        for (unsigned int i=0; i<lines; i++)
        {
            for (unsigned int j=0; j<nbrCols; j++)
            {
                setValue(matrix[(i*nbrCols)+j], i, j);
            }
        }
    }
}

void Matrix::setIdentity()
{
    for (unsigned int i=0; i<lines; i++)
    {
        for (unsigned int j=0; j<columns; j++)
        {
            if (i == j)
            {
                setValue(1.0, i, j);
            }
            else
            {
                setValue(0.0, i, j);
            }
        }
    }
}

vector<double> Matrix::returnMatrix() const
{
    return grid;
}

unsigned int Matrix::returnNbrLines() const
{
    return lines;
}

unsigned int Matrix::returnNbrColumns() const
{
    return columns;
}

bool Matrix::isSquareMat() const
{
    return squareMatrix;
}

void Matrix::printMatrix() const
{
    for (unsigned int i=0; i<lines; i++)
    {
        for (unsigned int j=0; j<columns; j++)
        {
            cout << value(i, j) << " ";
        }
        cout << endl;
    }
}

Matrix& Matrix::operator=(const Matrix &srcMatrix)
{
    if (this != &srcMatrix)
    {
        lines=srcMatrix.lines;
        columns=srcMatrix.columns;
        squareMatrix=srcMatrix.squareMatrix;
        grid = srcMatrix.grid;
    }
    return *this;
}

Matrix& Matrix::operator+=(const Matrix &matrix2)
{
    if (matrix2.lines == lines and matrix2.columns == columns)
    {
        for (unsigned int i=0; i<lines; i++)
        {
            for (unsigned int j=0; j<columns; j++)
            {
                setValue(value(i,j)+matrix2.value(i,j), i, j);
            }
        }
    }
    return *this;
}

Matrix& Matrix::operator-=(const Matrix &matrix2)
{
    return operator+=(Matrix(matrix2)*=-1);
}

Matrix& Matrix::operator*=(const double &cst)
{
    for (unsigned int i=0; i<lines; i++)
    {
        for (unsigned int j=0; j<columns; j++)
        {
            setValue(value(i, j)*cst, i, j);
        }
    }
    return *this;
}

Matrix& Matrix::operator/=(const double &cst)
{
    return operator*=(1/cst);
}

Matrix& Matrix::operator*=(const Matrix &matrix2)
{
    Matrix matrix1(*this);

    double currentValue;
    if (matrix1.columns == matrix2.lines)
    {
        for (unsigned int i=0; i<matrix1.lines; i++)
        {
            for (unsigned int j=0; j<matrix2.columns; j++)
            {
                currentValue=0;
                for (unsigned int k=0; k<matrix1.columns; k++)
                {
                    currentValue+=matrix1.value(i,k)*matrix2.value(k, j);
                }
                setValue(currentValue, i, j);
            }
        }
    }
    // Changer la taille de la matrice
    for (unsigned int i=returnNbrLines(); i>0; i--)
    {
        grid.erase(grid.begin()+(i-1)*returnNbrColumns()+matrix2.returnNbrColumns(), grid.begin()+i*returnNbrColumns());
    }
    columns=matrix2.returnNbrColumns();
    return *this;
}

Matrix Matrix::pow(int exp)
{
    Matrix result(*this);
    if (exp>0)
    {
        for (int i=1; i<exp; i++)
        {
            result*=*this;
        }
    }
    else if (exp<0)
    {
        result.invert();
        Matrix inverted(result);
        for (int i=1; i<-exp; i++)
        {
            result*=inverted;
        }
    }
    else
    {
        result.setIdentity();
    }
    return result;
}

Matrix Matrix::systemSolve(const Matrix &indepMat)
{
    Matrix inverted(*this);
    inverted.invert();
    return inverted*indepMat;
}

void Matrix::transpose()
{
    if (isSquareMat())
    {
        Matrix cpy(*this);
        for (unsigned int i=0; i<lines; i++)
        {
            for (unsigned int j=0; j<columns; j++)
            {
                setValue(cpy.value(j,i), i, j);
            }
        }
    }
}

void Matrix::diagonalise()
{
    if (isSquareMat())
    {
        double lineCoef;
        for (unsigned int i=0; i<returnNbrLines(); i++) //Pour chaque ligne
        {
            for (unsigned int j=0; j<returnNbrLines(); j++)
            {
                if (j != i)
                {
                    //if (value(i,i) != 0.0)
                    //{
                        lineCoef=value(j,i)/value(i,i);
                        for (unsigned int k=0; k<returnNbrColumns(); k++)
                        {
                            setValue(value(j,k)-(lineCoef*value(i,k)), j, k);
                        }
                    //}
                }
            }
        }
    }
}

void Matrix::triangulate()
{
    if (isSquareMat())
    {
        double lineCoef;
        for (unsigned int i=0; i<returnNbrLines(); i++) //Pour chaque ligne
        {
            for (unsigned int j=i+1; j<returnNbrLines(); j++)
            {
                lineCoef=value(j,i)/value(i,i);
                for (unsigned int k=0; k<returnNbrColumns(); k++)
                {
                    setValue(value(j,k)-(lineCoef*value(i,k)), j, k);
                }
            }
        }
    }
}

double Matrix::trace() const
{
    double sum=0;
    if (isSquareMat())
    {
        for (unsigned int i=0; i<returnNbrLines(); i++)
        {
            sum+=value(i,i);
        }
    }
    return sum;
}

double Matrix::diagonalMult() const
{
    if (isSquareMat())
    {
        double mult=1;
        for (unsigned int i=0; i<returnNbrLines(); i++)
        {
            mult*=value(i,i);
        }
        return mult;
    }
    return 0;
}

double Matrix::determinant() const
{
    if (isSquareMat())
    {
        double determ=0;
        Matrix temp(*this);
        temp.diagonalise();
        determ=temp.diagonalMult();
        return determ;
    }
    return 0.0;
}

void Matrix::invert()
{
    if (isSquareMat())
    {
        Matrix temp(*this);
        setIdentity();
        double lineCoef;
        for (unsigned int i=0; i<temp.returnNbrLines(); i++) //Pour chaque ligne
        {
            for (unsigned int j=0; j<temp.returnNbrLines(); j++)
            {
                if (j != i)
                {
                    lineCoef=temp.value(j,i)/temp.value(i,i);
                    for (unsigned int k=0; k<temp.returnNbrColumns(); k++)
                    {
                        temp.setValue(temp.value(j,k)-(lineCoef*temp.value(i,k)), j, k);
                        setValue(value(j,k)-(lineCoef*value(i,k)), j, k);
                    }
                }
            }
        }
        for (unsigned int i=0; i<temp.returnNbrLines(); i++)
        {
            if (temp.value(i,i) != 1)
            {
                for (unsigned int j=0; j<returnNbrColumns(); j++)
                {
                    setValue(value(i,j)/temp.value(i,i), i, j);
                }
            }
        }
    }
}

void Matrix::save(string filename) const
{
    ofstream file(filename.c_str());
    if (file.is_open())
    {
        //Lignes,colonnes
        //vecteur (ex : "1 67 -9 4")
        file << returnNbrLines() << "," << returnNbrColumns() << endl;
        for (unsigned int i=0; i<returnNbrColumns()*returnNbrLines(); i++)
        {
            file << grid[i] << " ";
        }
        file.close();
    }
}

void Matrix::load(string filename)
{
    ifstream file(filename.c_str());
    if (file.is_open())
    {
        file >> lines;
        file.seekg(1, file.cur);
        file >> columns;
        squareMatrix=(lines==columns);
        grid.resize(lines*columns);
        for (unsigned int i=0; i<lines*columns; i++)
        {
            file >> grid[i];
            file.seekg(1, file.cur);
        }
        file.close();
    }
}

Matrix::~Matrix()
{

}

//Opérateurs surchargés
Matrix operator+(const Matrix &matrix1, const Matrix &matrix2)
{
    Matrix result(matrix1);
    result+=matrix2;
    return result;
}

Matrix operator-(const Matrix &matrix1, const Matrix &matrix2)
{
    Matrix result(matrix1);
    result-=matrix2;
    return result;
}

Matrix operator*(const double &cst, const Matrix &matrix)
{
    Matrix result(matrix);
    result*=cst;
    return result;
}

Matrix operator*(const Matrix &matrix, const double &cst)
{
    return operator*(cst, matrix);
}

Matrix operator/(const Matrix &matrix, const double &cst)
{
    Matrix result(matrix);
    result/=cst;
    return result;
}

Matrix operator*(const Matrix &matrix1, const Matrix &matrix2)
{
    Matrix result(matrix1);
    result*=matrix2;
    return result;
}
