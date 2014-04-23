#include "action.h"
#include <iostream>
#include <cstdlib>

Action::Action(Boat *b, QObject *parent) :
    QObject(parent)
{
    boat = b;
    simulationTimer = new QTimer(this);
    infoTimer = new QTimer(this);
    connect(simulationTimer, SIGNAL(timeout()), this, SLOT(tick()));
    connect(infoTimer, SIGNAL(timeout()), this, SLOT(showInfo()));
}

void Action::startSimulation()
{
    simulationTimer->start(1);
}

void Action::startInfoTimer(int msec)
{
    infoTimer->start(msec);
}

void Action::stopTimers()
{
    simulationTimer->stop();
    infoTimer->stop();
}

void Action::showInfo()
{
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
    std::cout<<"Current velocity: "<<boat->getBoatVelocity()<<"\n";
}

void Action::tick()
{
    boat->step();
}
