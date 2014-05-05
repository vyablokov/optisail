#include <QCoreApplication>
#include "action.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    double mass = 75, sailSurface = 10, s = 1, azimuth = 0;
    Nature *world = new Nature();
    Boat *boat = new Boat(mass, sailSurface, s, azimuth, world);
    Action *a = new Action(boat);

    world->setWindAngle(330);
    boat->setRudderAngle(0);
    //boat->optimizeSailAngle(int(world->getWindAngleN()));
    boat->setSailToBoatAngle(0);

    a->startSimulation();
    a->startInfoTimer(200);

    /*char cmd;
    while(1)
    {
        cmd = getchar();
        switch(cmd)
        {
        case 'p':
            if(a->simulationStarted)
            {
                a->stopTimers();
                std::cout<<"Paused.\n";
            }
            else
            {
                a->startSimulation();
                std::cout<<"Resuming.\n";
                a->startInfoTimer(200);
            }
            break;
        case 'q':
            a->stopTimers();
            std::cout<<"Goodbye.\n";
            exit(0);
        default:
            break;
        }
    }*/

    return app.exec();
}
