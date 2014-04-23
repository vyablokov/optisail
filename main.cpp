#include <QCoreApplication>
#include "action.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    double mass = 75, sailSurface = 10, s = 1.4, azimuth = 0;
    Nature *world = new Nature();
    Boat *testBoat = new Boat(mass, sailSurface, s, azimuth, world);
    Action *a = new Action(testBoat);

    testBoat->setX(0);
    testBoat->setY(0);

    a->startSimulation();
    a->startInfoTimer(200);

    return app.exec();
}
