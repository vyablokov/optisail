#ifndef BOAT_H
#define BOAT_H

#include "interpolation.h"
#include "nature.h"

class Boat
{
    double mass;
    double sailSurface;
    double s;
    double azimuth;
    double rudderAngle;
    double sailToBoatAngle;
    QString sailToBoatOrientation;

    double attackAngle;
    QString sailSide;

    double boatVelocity;
    double a;

    double frictionForce;
    double tractionForce;
    double waterResistForce;


    double x;
    double y;

    Nature *World;
    Interpolation *Cx;
    Interpolation *Cy;

    void turnSail(QString dir);
public:
    Boat(double m, double sailSurf, double s1, double azimuth1, Nature *N);

    double getM ()  {return mass;}
    double getSailS () {return sailSurface;}
    double getS ()  {return s;}
    double getAzimuth () {return azimuth;}
    double getX () {return x;}
    double getY() {return y;}
    double getVBoat() { return boatVelocity;}
    double getA() {return a;}
    double getSailAngle() { return sailToBoatAngle; }
    double getFFriction() { return frictionForce; }
    double getFTraction() { return tractionForce; }
    double getFWaterResist() { return waterResistForce; }

    QString getSailSide() { return sailSide; }
    double getAttackAngle() { return attackAngle; }
    QString getSailToBoatOrientation() { return sailToBoatOrientation; }

    double getRudderAngle() { return rudderAngle; }

    void setSailSurface(double s) { sailSurface = s; }
    void setS(double s1) { s = s1; }
    void setM(double m) { mass = m; }

    void setAzimuth (double az) { azimuth = az; }
    void setBoatVelocity (double v) { boatVelocity = v;}
    void setA (double a1) {a = a1;}

    void setSailToBoatAngle (double angle) { sailToBoatAngle = angle;}
    void setSailToBoatOrientation(QString orientation) { sailToBoatOrientation = orientation; }

    void setX(double x1) { x = x1;}
    void setY(double y1) { y = y1;}
    void setRudderAngle(double angle) { rudderAngle = angle; }

    double decRudderAngle();
    double incRudderAngle();
    void optimizeSailAngle (int windAngle);
    double boatSpeed ();
    double sailAzimuth();
    void updateSailOrientation();
    void calculateForces();
    void step();
    void changeAzimuth();
    void turnSailRight() { turnSail("R"); }
    void turnSailLeft() { turnSail("L"); }
};

#endif // BOAT_H
