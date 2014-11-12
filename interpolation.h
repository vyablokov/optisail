//A class to interpolate phase characteristics of boat coefficients
#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <QVector>

class Interpolation
{
    QVector<double> *table;
//    int n;
    int step;
    double rightLimit;
    double leftLimit;

public:
    Interpolation(QVector<double> *c, int n, int s, double ll, double rl);
    double get (double angle);
};

#endif // INTERPOLATION_H
