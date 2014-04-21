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

    QVector<double> *cx  = new QVector<double>(37);
    cx->insert(0, 0.1);
    cx->insert(1, 0.1);
    cx->insert(2, 0.15);
    cx->insert(3, 0.18);
    cx->insert(4, 0.23);
    cx->insert(5, 0.3);
    cx->insert(6, 0.4);
    cx->insert(7, 0.6);
    cx->insert(8, 0.7);
    cx->insert(9, 0.8);
    cx->insert(10, 0.85);
    cx->insert(11, 0.93);
    cx->insert(12, 1);
    cx->insert(13, 1.07);
    cx->insert(14, 1.12);
    cx->insert(15, 1.19);
    cx->insert(16, 1.2);
    cx->insert(17, 1.23);
    cx->insert(18, 1.25);
    cx->insert(19, 1.23);
    cx->insert(20, 1.2);
    cx->insert(21, 1.19);
    cx->insert(22, 1.12);
    cx->insert(23, 1.07);
    cx->insert(24, 1);
    cx->insert(25, 0.93);
    cx->insert(26, 0.85);
    cx->insert(27, 0.8);
    cx->insert(28, 0.7);
    cx->insert(29, 0.6);
    cx->insert(30, 0.4);
    cx->insert(31, 0.3);
    cx->insert(32, 0.23);
    cx->insert(33, 0.18);
    cx->insert(34, 0.15);
    cx->insert(35, 0.1);
    cx->insert(36, 0.1);

    Cx = new Interpolation (cx, cx->size(), 5, 0, 180);

    QVector<double> *cy = new QVector<double>(37);
    cy->insert(0, 0);
    cy->insert(1, 0.3);
    cy->insert(2, 0.5);
    cy->insert(3, 0.9);
    cy->insert(4, 1.2);
    cy->insert(5, 1.4);
    cy->insert(6, 1.6);
    cy->insert(7, 1.3);
    cy->insert(8, 1.14);
    cy->insert(9, 1.09);
    cy->insert(10, 1);
    cy->insert(11, 0.87);
    cy->insert(12, 0.78);
    cy->insert(13, 0.66);
    cy->insert(14, 0.55);
    cy->insert(15, 0.4);
    cy->insert(16, 0.3);
    cy->insert(17, 0.2);
    cy->insert(18, 0);
    cy->insert(19, 0.3);
    cy->insert(20, 0.35);
    cy->insert(21, 0.4);
    cy->insert(22, 0.45);
    cy->insert(23, 0.5);
    cy->insert(24, 0.6);
    cy->insert(25, 0.8);
    cy->insert(26, 1.0);
    cy->insert(27, 1.2);
    cy->insert(28, 1.1);
    cy->insert(29, 0.9);
    cy->insert(30, 0.8);
    cy->insert(31, 0.7);
    cy->insert(32, 0.4);
    cy->insert(33, 0.35);
    cy->insert(34, 0.2);
    cy->insert(35, 0.1);
    cy->insert(36, 0);

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
