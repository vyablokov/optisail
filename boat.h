#ifndef BOAT_H
#define BOAT_H

#include "interpolation.h"
#include "nature.h"
#include <QString>

double _null;
double m;
double sailSurface;
double s;
double azimuth;
double rudderAngle;
double sailToBoatAngle;
QString sailToBoatOrintation;

double attackAngle;
QString sailSide;

double boatVelocity;
double a;

double frictionForce;
double tractionForce;
double waterResistForce;


double x;
double y;

Nature World;

Interpolation Cx;
Interpolation Cy;

class Boat
{
public:
    Boat(double m1, double sailSurf, double s1, double azimuth1, Nature N);
};

#endif // BOAT_H
