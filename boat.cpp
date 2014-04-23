#include "boat.h"

Boat::Boat (double m, double sailSurf, double s1, double azimuth1, Nature *N)
{
    World = N;
    mass = m;
    sailSurface = sailSurf;
    s = s1;
    azimuth = azimuth1;
    boatVelocity = 0;
    a = 0;

    x = 36.0;
    y = 55.0;

    sailToBoatAngle = 0;
    sailToBoatOrientation = "0";

    updateSailOrientation();

    rudderAngle = 0;

    QVector<double> *cx  = new QVector<double>();
    cx->push_back(0.1);
    cx->push_back(0.1);
    cx->push_back(0.15);
    cx->push_back(0.18);
    cx->push_back(0.23);
    cx->push_back(0.3);
    cx->push_back(0.4);
    cx->push_back(0.6);
    cx->push_back(0.7);
    cx->push_back(0.8);
    cx->push_back(0.85);
    cx->push_back(0.93);
    cx->push_back(1);
    cx->push_back(1.07);
    cx->push_back(1.12);
    cx->push_back(1.19);
    cx->push_back(1.2);
    cx->push_back(1.23);
    cx->push_back(1.25);
    cx->push_back(1.23);
    cx->push_back(1.2);
    cx->push_back(1.19);
    cx->push_back(1.12);
    cx->push_back(1.07);
    cx->push_back(1);
    cx->push_back(0.93);
    cx->push_back(0.85);
    cx->push_back(0.8);
    cx->push_back(0.7);
    cx->push_back(0.6);
    cx->push_back(0.4);
    cx->push_back(0.3);
    cx->push_back(0.23);
    cx->push_back(0.18);
    cx->push_back(0.15);
    cx->push_back(0.1);
    cx->push_back(0.1);
//    fprintf(stdout, "%d\n", cx->size());

    Cx = new Interpolation (cx, cx->size(), 5, 0, 180);

    QVector<double> *cy = new QVector<double>();
    cy->push_back(0);
    cy->push_back(0.3);
    cy->push_back(0.5);
    cy->push_back(0.9);
    cy->push_back(1.2);
    cy->push_back(1.4);
    cy->push_back(1.6);
    cy->push_back(1.3);
    cy->push_back(1.14);
    cy->push_back(1.09);
    cy->push_back(1);
    cy->push_back(0.87);
    cy->push_back(0.78);
    cy->push_back(0.66);
    cy->push_back(0.55);
    cy->push_back(0.4);
    cy->push_back(0.3);
    cy->push_back(0.2);
    cy->push_back(0);
    cy->push_back(0.3);
    cy->push_back(0.35);
    cy->push_back(0.4);
    cy->push_back(0.45);
    cy->push_back(0.5);
    cy->push_back(0.6);
    cy->push_back(0.8);
    cy->push_back(1.0);
    cy->push_back(1.2);
    cy->push_back(1.1);
    cy->push_back(0.9);
    cy->push_back(0.8);
    cy->push_back(0.7);
    cy->push_back(0.4);
    cy->push_back(0.35);
    cy->push_back(0.2);
    cy->push_back(0.1);
    cy->push_back(0);

    Cy = new Interpolation (cy, cy->size(), 5, 0, 180);
}

double Boat::decRudderAngle()
{
    if (rudderAngle > -30) --rudderAngle;
    return rudderAngle;
}

double Boat::incRudderAngle()
{
    if (rudderAngle < 30) ++rudderAngle;
    return rudderAngle;
}

void Boat::optimizeSailAngle (int windAngle)
{
    double speed = 0;
    azimuth = 0;
    World->setWindAngle(windAngle);

    sailToBoatOrientation = "L";
    sailToBoatAngle = 0;
    double nextSpeed = boatSpeed();
    do
    {
        sailToBoatAngle += 5;
        speed = nextSpeed;
        nextSpeed = boatSpeed();
    } while (speed <= nextSpeed && sailToBoatAngle < 90);
    sailToBoatAngle -= 5;
    double leftMax = sailToBoatAngle;
    double leftMaxVelocity = speed;

    sailToBoatOrientation = "R";
    sailToBoatAngle = 0;
    nextSpeed = boatSpeed();
    do
    {
        sailToBoatAngle += 5;
        speed = nextSpeed;
        nextSpeed = boatSpeed();
    } while (speed <= nextSpeed && sailToBoatAngle < 90);
    sailToBoatAngle -= 5;
    double rightMax = sailToBoatAngle;
    double rightMaxVelocity = speed;

    if (leftMaxVelocity >= rightMaxVelocity)
    {
        sailToBoatAngle = leftMax;
        sailToBoatOrientation = "L";
        boatVelocity = leftMaxVelocity;
    }
    else
    {
        sailToBoatAngle = rightMax;
        sailToBoatOrientation = "R";
        boatVelocity = rightMaxVelocity;
    }
}

double Boat::boatSpeed ()
{
    double delta_a = 0.001;
    boatVelocity = 0;
    a = 0;

    do
    {
        step();
    } while (a > delta_a);

    return boatVelocity;
}

double Boat::sailAzimuth()
{
    if (sailToBoatOrientation == "L")
        return World->add360(azimuth, sailToBoatAngle);
    else
        return World->sub360(azimuth, sailToBoatAngle);
}

void Boat::updateSailOrientation()
{
    attackAngle = World->sub360(sailAzimuth(), World->getWindAngle(boatVelocity, azimuth));

    if (attackAngle > 180)
    {
        sailSide = "Out";
        attackAngle = 360 - attackAngle;
    }
    else sailSide = "In";

    if (sailToBoatOrientation == "L")
    {
        if (sailSide == "In") sailSide = "Out";
        else sailSide = "In";
    }
}

void Boat::calculateForces()
{
    waterResistForce = -World->getWaterResistForce(boatVelocity, s);
    if (boatVelocity < 0) waterResistForce *= -1;

    frictionForce = World->getFrictionForce(Cx->get(attackAngle), sailSurface, World->getWindVelocity(boatVelocity, azimuth));
    tractionForce = World->getTractionForce(Cy->get(attackAngle), sailSurface, World->getWindVelocity(boatVelocity, azimuth));
    if (getSailSide() == "Out") tractionForce *= -1;
}

void Boat::step()
{
    changeAzimuth();
    updateSailOrientation();
    calculateForces();

    a = (
        World->getSinFromDegrees(sailToBoatAngle) * tractionForce
        - frictionForce * World->getCosFromDegrees(World->getWindAngleToBoat(azimuth))
        + waterResistForce
        ) / mass;


    double delta_t = 0.001;
    x += (boatVelocity * delta_t + a * delta_t * delta_t / 2) * World->getSinFromDegrees(azimuth) / 100000.0;
    y -= (boatVelocity * delta_t + a * delta_t * delta_t / 2) * World->getCosFromDegrees(azimuth) / 100000.0;

    boatVelocity += a * delta_t;
}

void Boat::changeAzimuth()
{
    double c = 2000;
    double delta_azimuth = boatVelocity / 100000.0 * rudderAngle * c;
    azimuth = World->add360(azimuth, delta_azimuth);
}

void Boat::turnSail(QString dir)
{
    QString opdir = "L";
    if (dir == "L") opdir = "R";

    if (sailToBoatOrientation == dir && sailToBoatAngle < 90)
        ++sailToBoatAngle;

    else if (sailToBoatOrientation == "0")
    {
        sailToBoatAngle = 1;
        sailToBoatOrientation = dir;
    }

    else if (sailToBoatOrientation == opdir)
    {
        --sailToBoatAngle;
        if (sailToBoatAngle == 0) sailToBoatOrientation = "0";
    }
}
