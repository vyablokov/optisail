#include <QCoreApplication>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include "boat.h"

#define DELTA_X 5
#define DELTA_F 0.1
#define k 0.05

double mass = 75, sailSurface = 10, s = 2;
Nature *world;
Boat *boat;
int windAngle = 105;

double fmin (double a, double b)
{
    if(a > b) return b;
    else return a;
}

double fmax (double a, double b)
{
    if(a > b) return a;
    else return b;
}

// Проверка вхождения точки в пределы многоугольника (1 если входит, 0 если нет)
int pointInPoly( double pgon[][2], int numverts, double* point )
{
    int i, crossings = 0;

    for ( i = 0; i < numverts; i++ ) {

        double x1 = pgon[i][0];
        double y1 = pgon[i][1];
        double x2 = pgon[(i + 1) % numverts][0];
        double y2 = pgon[(i + 1) % numverts][1];
        double d = (point[1] - y1) * (x2 - x1) - (point[0] - x1) * (y2 - y1);

        if ( (y1 >= point[1]) != (y2 >= point[1]) ) {
            crossings += y2 - y1 >= 0 ? d >= 0 : d <= 0;
        }

        if ( !d && fmin( x1, x2 ) <= point[0] && point[0] <= fmax( x1, x2 )
                && fmin( y1, y2 ) <= point[1] && point[1] <= fmax( y1, y2 ) ) {
            return 1;
        }
    }
    return crossings & 1;
}

// Расстояние между точками
double calcDistance(double* startPoint, double* destPoint) {
    return sqrt((destPoint[1] - startPoint[1])*(destPoint[1] - startPoint[1]) + (destPoint[0] - startPoint[0])*(destPoint[0] - startPoint[0]));
}

// Азимут от точки до точки
int calcAzimuth(double* startPoint, double* destPoint) {
    return 90 - world->toDegrees(asin((destPoint[1] - startPoint[1]) / calcDistance(startPoint, destPoint)));
}

// Угол ветра относительно азимута
int calcRelativeWindAngle(double* startPoint, double* destPoint)
{
    return world->sub360(windAngle, calcAzimuth(startPoint, destPoint));
}

// Время прямого пути от точки до точки
double calcTravelTime(double* startPoint, double* destPoint)
{
    boat->optimizeSailAngle(calcRelativeWindAngle(startPoint, destPoint));
    double travelTime = calcDistance(startPoint, destPoint) / boat->getBoatVelocity();
    if(travelTime < 0)
    {
        std::cout<<"Whoops! velocity is "<<boat->getBoatVelocity()<<
        " on ["<<startPoint[0]<<";"<<startPoint[1]<<"] => ["<<destPoint[0]<<";"<<destPoint[1]<<"]"<<
        " when rel. wind angle is "<<calcRelativeWindAngle(startPoint, destPoint)<<"\n";
    }
    return travelTime;
}

// Общее время пути до и после поворота
double calcTotalTime(double* startPoint, double* turnPoint, double* destPoint)
{
    return calcTravelTime(startPoint, turnPoint) + calcTravelTime(turnPoint, destPoint);
}

// Проверка на слишком острый курс
bool tooSharpCourse(double* startPoint, double* destPoint)
{
    int relativeWindAngle = calcRelativeWindAngle(startPoint, destPoint);
    if(relativeWindAngle < 30 || relativeWindAngle > 330)
        return true;
    else
        return false;
}

// Определение оптимальной точки поворота
void getTurnPoint(double* bestTurnPoint, double* startPoint, double* destPoint, double badTravelTime) {
    double totalTime = badTravelTime;
    double bestTime;
    double curTurnPoint[2];
    bestTurnPoint[0] = destPoint[0];
    bestTurnPoint[1] = destPoint[1];
    bool firstIter = true;

    //Method I (равномерный поиск)
    double leftBound, rightBound, upBound, downBound;
    double margin = calcDistance(startPoint, destPoint) * 2;
    if (startPoint[0] < destPoint[0])
    {
        leftBound = startPoint[0] - margin;
        rightBound = destPoint[0] + margin;
    }
    else
    {
        leftBound = destPoint[0] - margin;
        rightBound = startPoint[0] + margin;
    }
    if (startPoint[1] < destPoint[1])
    {
        downBound = startPoint[1] - margin;
        upBound = destPoint[1] + margin;
    }
    else
    {
        downBound = destPoint[1] - margin;
        upBound = startPoint[1] + margin;
    }
    double step_x = fabs((rightBound - leftBound) / 5);
    double step_y = fabs((upBound - downBound) / 5);

    std::ofstream surface;
    surface.open("surface.txt", std::ios::out);
    surface<<std::fixed<<std::setprecision(2);

    while(step_x > DELTA_X/2 && step_y > DELTA_X/2)
    {
        for (curTurnPoint[0] = leftBound; curTurnPoint[0] < rightBound; curTurnPoint[0]+=step_x)
            for (curTurnPoint[1] = downBound; curTurnPoint[1] < upBound; curTurnPoint[1]+=step_y)
            {
                if(tooSharpCourse(startPoint, curTurnPoint) || tooSharpCourse(curTurnPoint, destPoint))
                {
                    continue;
                }
                totalTime = calcTotalTime(startPoint, curTurnPoint, destPoint);
                surface<<curTurnPoint[0]<<"\t"<<curTurnPoint[1]<<"\t"<<totalTime<<"\n\n";
                if(firstIter)
                {
                    bestTime = totalTime;
                    firstIter = false;
                }
                if (totalTime < bestTime)
                {
                    bestTime = totalTime;
                    bestTurnPoint[0] = curTurnPoint[0];
                    bestTurnPoint[1] = curTurnPoint[1];
                }

            }
        leftBound = bestTurnPoint[0] - step_x;
        rightBound = bestTurnPoint[0] + step_x;
        upBound = bestTurnPoint[1] + step_y;
        downBound = bestTurnPoint[1] - step_y;
        step_x = fabs((rightBound - leftBound) / 5);
        step_y = fabs((upBound - downBound) / 5);
    }
    surface.close();
    /////////////////

    //Method II (случ. поиск с пост. радиусом + метод Паулла)
    /*double radius;
    double prevTotalTime = 0;
    int azimuth = 0;
    double bestAzimuth = 0;
    double point1[2], point2[2], point3[2], sqRootPoint[2];
    double time1 = 0, time2 = 0, time3 = 0;
    double x1=0, x2=0, x3=0, xSqRoot=0, e23=0, e13=0, e12=0, r23=0, r13=0, r12=0;
    int sign=1;
    double prevTurnPoint[2];
    sqRootPoint[0] = startPoint[0];
    sqRootPoint[1] = startPoint[1];
    //srand(time(NULL));

    do
    {
        prevTotalTime = totalTime;
        prevTurnPoint[0] = sqRootPoint[0];
        prevTurnPoint[1] = sqRootPoint[1];
        radius = calcDistance(prevTurnPoint, destPoint) / 10;
        firstIter = true;

        for(int i = 0; i < 30; i++) //выбираем направление поиска из случайных точек на окружности
        {
            azimuth = rand() % 360 ;
            curTurnPoint[0] = prevTurnPoint[0] + radius * world->getCosFromDegrees(90 - azimuth);
            curTurnPoint[1] = prevTurnPoint[1] + radius * world->getSinFromDegrees(90 - azimuth);

            if(tooSharpCourse(startPoint, curTurnPoint) || tooSharpCourse(curTurnPoint, destPoint))
            {
                continue;
            }
            totalTime = calcTotalTime(startPoint, curTurnPoint, destPoint);
            if(firstIter)
            {
                bestTime = totalTime;
                firstIter = false;
            }
            if (totalTime < bestTime)
            {
                bestTime = totalTime;
                bestAzimuth = azimuth;
                bestTurnPoint[0] = curTurnPoint[0];
                bestTurnPoint[1] = curTurnPoint[1];
            }
        }

        point1[0] = bestTurnPoint[0];   //метод Паулла
        point1[1] = bestTurnPoint[1];
        point2[0] = point1[0] + k * radius * world->getCosFromDegrees(90 - bestAzimuth);
        point2[1] = point1[1] + k * radius * world->getSinFromDegrees(90 - bestAzimuth);
        time1 = calcTotalTime(startPoint, point1, destPoint);
        time2 = calcTotalTime(startPoint, point2, destPoint);
        if (time1 > time2)
        {
            point3[0] = point1[0] + 2 * k * radius * world->getCosFromDegrees(90 - bestAzimuth);
            point3[1] = point1[1] + 2 * k * radius * world->getSinFromDegrees(90 - bestAzimuth);
        }
        else
        {
            point3[0] = point1[0] - k * radius * world->getCosFromDegrees(90 - bestAzimuth);
            point3[1] = point1[1] - k * radius * world->getSinFromDegrees(90 - bestAzimuth);
        }

        do
        {
            time1 = calcTotalTime(startPoint, point1, destPoint);
            time2 = calcTotalTime(startPoint, point2, destPoint);
            time3 = calcTotalTime(startPoint, point3, destPoint);
            if(calcAzimuth(prevTurnPoint, point1) != bestAzimuth) sign = -1;
            else sign = 1;
            x1 = sign * calcDistance(prevTurnPoint, point1);
            if(calcAzimuth(prevTurnPoint, point2) != bestAzimuth) sign = -1;
            else sign = 1;
            x2 = sign * calcDistance(prevTurnPoint, point2);
            if(calcAzimuth(prevTurnPoint, point3) != bestAzimuth) sign = -1;
            else sign = 1;
            x3 = sign * calcDistance(prevTurnPoint, point3);
            e23 = x2*x2 - x3*x3;
            e13 = x1*x1 - x3*x3;
            e12 = x1*x1 - x2*x2;
            r23 = x2 - x3;
            r13 = x1 - x3;
            r12 = x1 - x2;
            xSqRoot = (time1*e23 - time2*e13 + time3*e12) / (time1*r23 - time2*r13 + time3*r12) / 2;
            sqRootPoint[0] = prevTurnPoint[0] + xSqRoot * world->getCosFromDegrees(90 - bestAzimuth);
            sqRootPoint[1] = prevTurnPoint[1] + xSqRoot * world->getSinFromDegrees(90 - bestAzimuth);
            totalTime = calcTotalTime(startPoint, sqRootPoint, destPoint);

            if(xSqRoot >= x2 && xSqRoot <= x3)
            {
                point1[0] = point2[0];
                point1[1] = point2[1];
                point2[0] = sqRootPoint[0];
                point2[1] = sqRootPoint[1];
                point3[0] = point3[0];
                point3[1] = point3[1];
                continue;
            }
            if(xSqRoot >= x1 && xSqRoot <= x2)
            {
                point1[0] = point1[0];
                point1[1] = point1[1];
                point3[0] = point2[0];
                point3[1] = point2[1];
                point2[0] = sqRootPoint[0];
                point2[1] = sqRootPoint[1];
                continue;
            }
            if(xSqRoot >= x3 && xSqRoot <= x1)
            {
                double temp[2];
                temp[0] = point1[0];
                temp[1] = point1[1];
                point1[0] = point3[0];
                point1[1] = point3[1];
                point2[0] = sqRootPoint[0];
                point2[1] = sqRootPoint[1];
                point3[0] = temp[0];
                point3[1] = temp[1];
                continue;
            }
            if(xSqRoot > x3 && x3 > x2)
            {
                point1[0] = point2[0];
                point1[1] = point2[1];
                point2[0] = point3[0];
                point2[1] = point3[1];
                point3[0] = sqRootPoint[0];
                point3[1] = sqRootPoint[1];
                continue;
            }
            if(xSqRoot < x3 && x3 < x1)
            {
                point3[0] = point2[0];
                point3[1] = point2[1];
                point2[0] = point1[0];
                point2[1] = point1[1];
                point1[0] = sqRootPoint[0];
                point1[1] = sqRootPoint[1];
                continue;
            }
        }
        while (calcDistance(point1, point3) > DELTA_X);
    }
    while (fabs(totalTime - prevTotalTime) > DELTA_F);*/
    ///////////////
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    int azimuth = 0;
    double startPoint[2] = {0, 0};
    double destPoint[2] = {0, 3000};
    double travelTime = 0x7ff0000000000000;
    world = new Nature();
    boat = new Boat(mass, sailSurface, s, azimuth, world);

    std::cout<<std::fixed<<std::setprecision(2);

    if(!tooSharpCourse(startPoint, destPoint))
    {
        travelTime = calcTravelTime(startPoint, destPoint);

        //std::cout<<calcRelativeWindAngle(startPoint, destPoint)<<"\n";

        std::cout<<"Parameters of straight course\n";
        std::cout<<"Distance: "<<calcDistance(startPoint, destPoint)<<" m\n";
        std::cout<<"Optimal sail angle: "<<boat->getSailAngle()<<" deg\n";
        std::cout<<"Sail to boat orientation: "<<boat->getSailToBoatOrientation().toStdString()<<"\n";
        std::cout<<"Max. velocity: "<<boat->getBoatVelocity()<<" m/s\n";
        std::cout<<"Estimated best travel time: "<<travelTime<<" s\n";
    }
    else
        std::cout<<"Straight course is to sharp to pass.\n";

    double point1[2];
    double point2[2];
    double point3[2];

    getTurnPoint(point1, startPoint, destPoint, travelTime);

    if (calcDistance(point1, destPoint) < 1)
        std::cout<<"Straight course is optimal.\n";
    else
    {
        std::cout<<"Best travel time for \"1-turn\" course is ";
        std::cout<<calcTotalTime(startPoint, point1, destPoint)<<" s\n";
        std::cout<<"Turn point is at ["<<point1[0]<<";"<<point1[1]<<"]\n";
    }


    /*getTurnPoint(point2, startPoint, destPoint);
    getTurnPoint(point1, startPoint, point2);
    getTurnPoint(point3, point2, destPoint);

    std::cout<<"Waypoints:\n";
    std::cout<<startPoint[0]<<"\t"<<startPoint[1]<<"\n";
    std::cout<<point1[0]<<"\t"<<point1[1]<<"\n";
    std::cout<<point2[0]<<"\t"<<point2[1]<<"\n";
    std::cout<<point3[0]<<"\t"<<point3[1]<<"\n";
    std::cout<<destPoint[0]<<"\t"<<destPoint[1]<<"\n";
    std::cout<<"Time: "<<calcTotalTime(startPoint, point1, point2) + calcTotalTime(point2, point3, destPoint)<<" s\n";*/

    std::ofstream waypoints;
    waypoints.open("waypoints.txt", std::ios::out);
    waypoints<<std::fixed<<std::setprecision(2);
    waypoints<<startPoint[0]<<"\t"<<startPoint[1]<<"\n";
    waypoints<<point1[0]<<"\t"<<point1[1]<<"\n";
    waypoints<<destPoint[0]<<"\t"<<destPoint[1]<<"\n";
    waypoints.close();

    exit(0);
    return app.exec();
}
