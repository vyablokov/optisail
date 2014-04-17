#include "nature.h"
#include <cmath>

#define PI 3.14

Nature::Nature ()
{
    airWeight = 1.29;
    waterWeight = 1000;
    windVelocity = 10;
    windAngle = 0;
    k = 2.0 * 0.01;
    useWindB = false;
    gAcceleration = 9.8;
}

double Nature::add360(double angle1, double angle2)
{
    double res = angle1 + angle2;
    if (res < 0) res += 360;
    if (res > 360) res -= 360;
    return res;
}

double Nature::getWindVelocityN ()
{
    return windVelocity;
}

double Nature::sub360(double angle1, double angle2)
{
    double res = angle1 - angle2;
    if (res < 0) res += 360;
    if (res > 360) res -= 360;
    return res;
}

double Nature::getWindAngleToBoat (double azimuth)
{
    return sub360(azimuth, windAngle);
}

double Nature::windToAzimuth (double wind)
{
    return add360(wind, 180);
}

double Nature::getWindAngle(double boatVelocity, double azimuth)
{
    if (useWindB) return getAngleB(boatVelocity, azimuth);
    else return getWindAngleN();
}

double Nature::getWindVelocity(double boatVelocity, double azimuth)
{
    if (useWindB) return getWindVelocityB(boatVelocity, azimuth);
    else return getWindVelocityN();
}

double Nature::getWindVelocityB(double boatVelocity, double azimuth)
{
    double windAngleToBoat = getWindAngleToBoat(azimuth);
    return sqrt(boatVelocity * boatVelocity + windVelocity * windVelocity
    - 2 * boatVelocity * windVelocity * getCosFromDegrees(180 - windAngleToBoat));
}

double Nature::getAngleB (double boatVelocity, double azimuth)
{
    if (boatVelocity == 0 || azimuth == windAngle ||
        fabs(azimuth - windAngle) == 180)
        return getWindAngleN();

//    if (getWindAngleToBoat(azimuth) > 0)
    if (getWindToBoatOrientation(azimuth) == "L")
        return getSafeArccos((getWindVelocityB (boatVelocity, azimuth) * getWindVelocityB (boatVelocity, azimuth)
        + boatVelocity * boatVelocity - windVelocity * windVelocity)
        / (2 * getWindVelocityB (boatVelocity, azimuth) * boatVelocity));
    else
        return 360.0 - getSafeArccos((getWindVelocityB(boatVelocity, azimuth) * getWindVelocityB(boatVelocity, azimuth)
        + boatVelocity * boatVelocity - windVelocity * windVelocity)
        / (2 * getWindVelocityB(boatVelocity, azimuth) * boatVelocity));
}

QString Nature::getWindToBoatOrientation(double azimuth)
{
    if (getWindAngleToBoat(azimuth) > 180) return "L";
    else return "R";
}

double Nature::getFrictionForce (double cx, double sailSurface, double windVel)
{
    return gAcceleration * cx * windVel * windVel * airWeight * sailSurface / 2.0;
}

double Nature::getTractionForce (double cy, double sailSurface, double windVel)
{
    return gAcceleration * cy * windVel * windVel * airWeight *  sailSurface / 2.0;
}

double Nature::getWaterResistForce(double boatVelocity, double s)
{
    return gAcceleration * k * boatVelocity * boatVelocity * waterWeight * s /2.0;
}

double Nature::toRadians (double angle)
{
    return angle * PI / 180.0;
}

double Nature::toDegrees(double angle)
{
    return angle * 180.0 / PI;
}

double Nature::getSafeArcsin(double sinOfAngle)
{
    double deltaAngle = 0.999;
    if (sinOfAngle <= deltaAngle)
        return toDegrees(asin(sinOfAngle));
    else
        return toDegrees(PI / 2.0);
}

double Nature::getSafeArccos(double cosOfAngle)
{
    if (cosOfAngle >= 1) return 0.0;
    else return toDegrees(acos(cosOfAngle));
}

double Nature::getCosFromDegrees(double angle)
{
    double rad_angle = toRadians(angle);
    return cos(rad_angle);
}

double Nature::getSinFromDegrees(double angle)
{
    double rad_angle = toRadians(angle);
    return sin(rad_angle);
}
