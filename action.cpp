#include "action.h"

Action::Action(Boat *b, QObject *parent) :
    QObject(parent)
{
    boat = b;
    simulationTimer = new QTimer(this);
    infoTimer = new QTimer(this);
    connect(simulationTimer, SIGNAL(timeout()), this, SLOT(tick()));
    connect(infoTimer, SIGNAL(timeout()), this, SLOT(showInfo()));
    simulationStarted = false;
    cmd = ' ';
    firstPoint = true;
    track.open("track.txt",std::ios::out);
}


void Action::startSimulation()
{
    simulationTimer->start(1);
    simulationStarted = true;
}

void Action::startInfoTimer(int msec)
{
    infoTimer->start(msec);
}

void Action::stopTimers()
{
    simulationTimer->stop();
    simulationStarted = false;
    infoTimer->stop();
}

void Action::writePointToTrack()
{
    if(!firstPoint)
        track.open("track.txt", std::ios::app);
    else
        firstPoint = false;
    track<<std::fixed<<std::setprecision(2);
    track<<boat->getX()<<"\t"<<boat->getY()<<"\n";
    track.close();
}

void Action::showInfo()
{
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
    std::cout<<std::fixed<<std::setprecision(2);
    std::cout<<"Velocity: "<<boat->getBoatVelocity()<<" m/s\n";
    std::cout<<"Acceleration: "<<boat->getA()<<" m/(s*s)\n";
    std::cout<<"FrictionForce: "<<boat->getFrictionForce()<<" N\n";
    std::cout<<"TractionForce: "<<boat->getTractionForce()<<" N\n";
    std::cout<<"WaterResistForce: "<<boat->getWaterResistForce()<<" N\n";
    std::cout<<"X: "<<boat->getX()<<" m\n";
    std::cout<<"Y: "<<boat->getY()<<" m\n";

    writePointToTrack();
}

void Action::tick()
{
    boat->step();
    switch(cmd)
    {
    case 'p':
        if(simulationStarted)
        {
            stopTimers();
        }
        else
        {
            startSimulation();
            startInfoTimer(200);
        }
        break;
    case 'q':
        stopTimers();
        return;
    default:
        break;
    }
}
