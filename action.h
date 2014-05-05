#ifndef ACTION_H
#define ACTION_H

#include <QObject>
#include <QTimer>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include "boat.h"

class Action : public QObject
{
    Q_OBJECT
    QTimer *simulationTimer, *infoTimer;
    Boat *boat;
    std::ofstream track;
    bool firstPoint;
public:
    bool simulationStarted;
    char cmd;

    explicit Action(Boat *b, QObject *parent = 0);
    void startSimulation();
    void startInfoTimer(int msec);
    void stopTimers();
    void writePointToTrack();

signals:

public slots:
    void tick();
    void showInfo();
};

#endif // ACTION_H
