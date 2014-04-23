//Simulation of physical conditions
#ifndef NATURE_H
#define NATURE_H

#include <QString>

class Nature
{
    double airWeight;
    double waterWeight;
    double windVelocity;
    double windAngle;
    double k;
    bool useWindB;
    double gAcceleration;
    double PI;
public:
    Nature();
    void setK(double kk) { k = kk; }
    void setWindKind(bool a) { useWindB = a; }
    void setwindVelocity(double windVel) { windVelocity = windVel; }

    double getWindVelocity() { return windVelocity; }
    double getK() { return k; }
    double getAirWeight () {return airWeight;}
    double getWaterWeight () {return waterWeight;}

    double getWindAngleN() {return windAngle;}
    void setWindAngle(double angle) { windAngle = angle; }

    double add360(double angle1, double angle2);
    double getWindVelocityN ();
    double sub360(double angle1, double angle2);
    double getWindAngleToBoat (double azimuth);
    double windToAzimuth (double wind);
    double getWindAngle(double boatVelocity, double azimuth);
    double getWindVelocity(double boatVelocity, double azimuth);
    double getWindVelocityB (double boatVelocity, double azimuth);
    double getAngleB (double boatVelocity, double azimuth);
    QString getWindToBoatOrientation(double azimuth);
    double getFrictionForce (double cx, double sailSurface, double windVel);
    double getTractionForce (double cy, double sailSurface, double windVel);
    double getWaterResistForce (double boatVelocity, double s);
    double toRadians (double angle);
    double toDegrees(double angle);
    double getSafeArcsin (double sinOfAngle);
    double getSafeArccos(double cosOfAngle);
    double getCosFromDegrees(double angle);
    double getSinFromDegrees(double angle);
};

#endif // NATURE_H
