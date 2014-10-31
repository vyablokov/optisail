#include <QCoreApplication>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include "boat.h"

#define RESOLUTION 4
#define DELTA_X 2
#define DELTA_F 0.1
#define k 2.0
#define RANDOM_POINTS 20
#define VERTEX_NUM 5
#define WAYPOINTS_FILE "waypoints.txt"

#define PRINT_WAYPOINTS
//#define USE_ADVANCED_METHOD

double mass = 75, sailSurface = 10, s = 2, fullTurnTime = 15;
Nature *world;
Boat *boat;
int windAngle;
std::ofstream waypoints;

double fmin2 (double a, double b)
{
    if(a > b) return b;
    else return a;
}

double fmax2 (double a, double b)
{
    if(a > b) return a;
    else return b;
}

double fmin3 (double a, double b, double c)
{
    return fmin2(fmin2(a, b), c);
}

double fmax3 (double a, double b, double c)
{
    return fmax2(fmax2(a, b), c);
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

        if ( !d && fmin2( x1, x2 ) <= point[0] && point[0] <= fmax2( x1, x2 )
                && fmin2( y1, y2 ) <= point[1] && point[1] <= fmax2( y1, y2 ) ) {
            return 1;
        }
    }
    return crossings & 1;
}

// Проверка пересечения отрезков
bool intersectLines (double* a, double* b, double* c, double* d)
{
    double common = (b[0] - a[0])*(d[1] - c[1]) - (b[1] - a[1])*(d[0] - c[0]);

    if (common == 0) return false;

    double rH = (a[1] - c[1])*(d[0] - c[0]) - (a[0] - c[0])*(d[1] - c[1]);
    double sH = (a[1] - c[1])*(b[0] - a[0]) - (a[0] - c[0])*(b[1] - a[1]);

    double r = rH / common;
    double s = sH / common;

    if (r >= 0 && r <= 1 && s >= 0 && s <= 1)
        return true;
    else
        return false;
}

// Проверка пересечения отрезка и окружности
bool intersectLineAndCircle    (   double* point1, double* point2, double* center, double radius)
{
    double p1[2], p2[2];
    p1[0] = point1[0] - center[0];
    p1[1] = point1[1] - center[1];
    p2[0] = point2[0] - center[0];
    p2[1] = point2[1] - center[1];


    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];

    //составляем коэффициенты квадратного уравнения на пересечение прямой и окружности.
    //если на отрезке [0..1] есть отрицательные значения, значит отрезок пересекает окружность
    double a = dx*dx + dy*dy;
    double b = 2.*(p1[0]*dx + p1[1]*dy);
    double c = p1[0]*p1[0] + p1[1]*p1[1] - radius*radius;

    //а теперь проверяем, есть ли на отрезке [0..1] решения
    if (-b < 0)
        return (c < 0);
    if (-b < (2.*a))
        return ((4.*a*c - b*b) < 0);

    return (a+b+c < 0);
}

// Вычисление геометрического ограничения (окружность)
double calcCircleRestriction(double* point, double* center, double radius)
{
    double r = (point[0] - center[0])*(point[0] - center[0]) + (point[1] - center[1])*(point[1] - center[1]) - radius*radius;
    return r;
}

// Вычисление барьерной функции
double barrierFunction(double* point, double* center, double radius)
{
    double g = calcCircleRestriction(point, center, radius);
    double phi = 0;
    double n = 2;
    if(g >= 0)
        phi = 1 / pow(g, n);
    else
        phi = INFINITY;
    return phi;
}

// Расстояние между точками
double calcDistance(double* startPoint, double* destPoint) {
    return sqrt((destPoint[1] - startPoint[1])*(destPoint[1] - startPoint[1]) + (destPoint[0] - startPoint[0])*(destPoint[0] - startPoint[0]));
}

// Азимут от точки до точки
int calcAzimuth(double* startPoint, double* destPoint) {
    int azimuth = world->sub360(90, world->toDegrees(acos((destPoint[0] - startPoint[0]) / calcDistance(startPoint, destPoint))));
    if(destPoint[1] < startPoint[1])
        azimuth = world->sub360(-1 * azimuth, 180);
    return azimuth;
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

// Расчет времени поворота
double calcTurnTime(double* startPoint, double* turnPoint, double* destPoint)
{
    return ( abs(calcAzimuth(turnPoint, destPoint) - calcAzimuth(startPoint, turnPoint)) / 360.0 ) * fullTurnTime;
}

// Общее время пути до и после поворота
double calcTotalTime(double* startPoint, double* turnPoint, double* destPoint)
{
    return calcTravelTime(startPoint, turnPoint) + calcTravelTime(turnPoint, destPoint) + calcTurnTime(startPoint, turnPoint, destPoint);
}

double calcTotalTimeWithRestriction(double* startPoint, double* turnPoint, double* destPoint, double* islandCenter, double islandRadius)
{
    return calcTotalTime(startPoint, turnPoint, destPoint) + barrierFunction(turnPoint, islandCenter, islandRadius);
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


// Определение оптимальной точки поворота простым способом
void getTurnPointSimple(double* bestTurnPoint, double* startPoint, double* destPoint, double badTravelTime, double* islandCenter, double islandRadius)
{
    //Method I (равномерный поиск)
    double totalTime = badTravelTime;
    double bestTime;
    double curTurnPoint[2];
    bestTurnPoint[0] = destPoint[0];
    bestTurnPoint[1] = destPoint[1];
    bool firstIter = true;
    double leftBound, rightBound, upBound, downBound;
    double margin = calcDistance(startPoint, destPoint) / 2;

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
    double step_x = fabs((rightBound - leftBound) / RESOLUTION);
    double step_y = fabs((upBound - downBound) / RESOLUTION);

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
                //totalTime = calcTotalTime(startPoint, curTurnPoint, destPoint);
                totalTime = calcTotalTimeWithRestriction(startPoint, curTurnPoint, destPoint, islandCenter, islandRadius);
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
        //surface<<"\n";
        leftBound = bestTurnPoint[0] - step_x;
        rightBound = bestTurnPoint[0] + step_x;
        upBound = bestTurnPoint[1] + step_y;
        downBound = bestTurnPoint[1] - step_y;
        step_x = fabs((rightBound - leftBound) / RESOLUTION);
        step_y = fabs((upBound - downBound) / RESOLUTION);
    }
    surface.close();
}

// Определение оптимальной точки поворота
void getTurnPoint(double* bestTurnPoint, double* startPoint, double* destPoint, double badTravelTime, double* islandCenter, double islandRadius)
{
    // Method II (случ. поиск с пост. радиусом + метод Паулла)
    double totalTime = badTravelTime;
    double bestTime;
    double curTurnPoint[2];
    bestTurnPoint[0] = destPoint[0];
    bestTurnPoint[1] = destPoint[1];
    bool firstIter = true;
    double radius;
    double prevTotalTime = 0;
    int azimuth = 0;
    double bestAzimuth = 0;
    double point1[2], point2[2], point3[2], sqRootPoint[2], prevBestPoint[2];
    double time1 = 0, time2 = 0, time3 = 0;
    double x1=0, x2=0, x3=0, xSqRoot=0, e23=0, e13=0, e12=0, r23=0, r13=0, r12=0;
    int sign=1;
    double prevTurnPoint[2];
    sqRootPoint[0] = startPoint[0];
    sqRootPoint[1] = startPoint[1];
    //srand(time(NULL));
    double radius_k = 25;

    do
    {
        prevTotalTime = totalTime;
        prevTurnPoint[0] = sqRootPoint[0];
        prevTurnPoint[1] = sqRootPoint[1];
        radius = calcDistance(startPoint, destPoint) / radius_k;
        firstIter = true;

        /*
        double a[2] = {0, 0}, b[2];
        int real_az = 0;
        for(;;)
        {
            azimuth = rand() % 360 ;
            b[0] = a[0] + 100 * world->getCosFromDegrees(world->sub360(90, azimuth));
            b[1] = a[1] + 100 * world->getSinFromDegrees(world->sub360(90, azimuth));
            real_az = calcAzimuth(a, b);
        }
        */

        for(int i = 0; i < RANDOM_POINTS; i++) //выбираем направление поиска из случайных точек на окружности
        {            
            azimuth = rand() % 360 ;
            curTurnPoint[0] = prevTurnPoint[0] + radius * world->getCosFromDegrees(world->sub360(90, azimuth));
            curTurnPoint[1] = prevTurnPoint[1] + radius * world->getSinFromDegrees(world->sub360(90, azimuth));
            //double real_az = calcAzimuth(prevTurnPoint, curTurnPoint);

            if(tooSharpCourse(startPoint, curTurnPoint) || tooSharpCourse(curTurnPoint, destPoint))
            {
                continue;
            }
            totalTime = calcTotalTimeWithRestriction(startPoint, curTurnPoint, destPoint, islandCenter, islandRadius);
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

        if (bestTime > prevTotalTime)
        {
            radius_k = radius_k / 2;
            continue;
        }

        point1[0] = bestTurnPoint[0];   //метод Паулла
        point1[1] = bestTurnPoint[1];
        point2[0] = point1[0] + k * radius * world->getCosFromDegrees(90 - bestAzimuth);
        point2[1] = point1[1] + k * radius * world->getSinFromDegrees(90 - bestAzimuth);
        time1 = calcTotalTimeWithRestriction(startPoint, point1, destPoint, islandCenter, islandRadius);
        time2 = calcTotalTimeWithRestriction(startPoint, point2, destPoint, islandCenter, islandRadius);
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
            time1 = calcTotalTimeWithRestriction(startPoint, point1, destPoint, islandCenter, islandRadius);
            time2 = calcTotalTimeWithRestriction(startPoint, point2, destPoint, islandCenter, islandRadius);
            time3 = calcTotalTimeWithRestriction(startPoint, point3, destPoint, islandCenter, islandRadius);

            if(time1 == fmin3(time1, time2, time3))
            {
                prevBestPoint[0] = point1[0];
                prevBestPoint[1] = point1[1];
            }
            if(time2 == fmin3(time1, time2, time3))
            {
                prevBestPoint[0] = point2[0];
                prevBestPoint[1] = point2[1];
            }
            if(time3 == fmin3(time1, time2, time3))
            {
                prevBestPoint[0] = point3[0];
                prevBestPoint[1] = point3[1];
            }

            //if(fabs(calcAzimuth(prevTurnPoint, point1) - bestAzimuth) > 90) sign = -1;
            if(abs(calcAzimuth(prevTurnPoint, point1) - bestAzimuth) > 2) sign = -1;
            else sign = 1;
            x1 = sign * calcDistance(prevTurnPoint, point1);
            if(abs(calcAzimuth(prevTurnPoint, point2) - bestAzimuth) > 2) sign = -1;
            else sign = 1;
            x2 = sign * calcDistance(prevTurnPoint, point2);
            if(abs(calcAzimuth(prevTurnPoint, point3) - bestAzimuth) > 2) sign = -1;
            else sign = 1;
            x3 = sign * calcDistance(prevTurnPoint, point3);
            e23 = x2*x2 - x3*x3;
            e13 = x1*x1 - x3*x3;
            e12 = x1*x1 - x2*x2;
            r23 = x2 - x3;
            r13 = x1 - x3;
            r12 = x1 - x2;
            xSqRoot = (time1*e23 - time2*e13 + time3*e12) / ((time1*r23 - time2*r13 + time3*r12) * 2.0);
            sqRootPoint[0] = prevTurnPoint[0] + xSqRoot * world->getCosFromDegrees(90 - bestAzimuth);
            sqRootPoint[1] = prevTurnPoint[1] + xSqRoot * world->getSinFromDegrees(90 - bestAzimuth);
            totalTime = calcTotalTimeWithRestriction(startPoint, sqRootPoint, destPoint, islandCenter, islandRadius);

            if (totalTime > fmin3(time1, time2, time3) || totalTime < 0)
            {
                totalTime = fmin3(time1, time2, time3);
                sqRootPoint[0] = prevBestPoint[0];
                sqRootPoint[1] = prevBestPoint[1];
                break;
            }

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
        //while (calcDistance(point1, point3) > DELTA_X);
        while (fabs(calcTotalTimeWithRestriction(startPoint, point2, destPoint, islandCenter, islandRadius) - calcTotalTimeWithRestriction(startPoint, point3, destPoint, islandCenter, islandRadius)) > DELTA_F);
    }
    while (fabs(totalTime - prevTotalTime) > DELTA_F && totalTime < prevTotalTime && radius > DELTA_X);
    bestTurnPoint = prevTurnPoint;
}

// Метафункция для выбора метода оптимизации
void optimizeTurn(double* bestTurnPoint, double* startPoint, double* destPoint, double badTravelTime, double* islandCenter, double islandRadius)
{
    // Получение курса с одним поворотом
#ifdef USE_ADVANCED_METHOD
    getTurnPoint(bestTurnPoint, startPoint, destPoint, badTravelTime, islandCenter, islandRadius);  // Метод случ. поиска + метод Паула
#endif
#ifndef USE_ADVANCER_METHOD
    getTurnPointSimple(bestTurnPoint, startPoint, destPoint, badTravelTime, islandCenter, islandRadius); // Метод равномерного поиска
#endif
}

// Рекурсивная оптимизация курса с одним поворотом с учетом препятствия и запись точек поворота в файл
double processRestrictions(double* startPoint, double* turnPoint, double* destPoint, double* islandCenter, double islandRadius)
{
    double newTurnPoint[2];
    double complexTravelTime = 0;

    if(intersectLineAndCircle(startPoint, turnPoint, islandCenter, islandRadius))
    {
        optimizeTurn(newTurnPoint, startPoint, turnPoint, 0x7ff0000000000000, islandCenter, islandRadius);
        processRestrictions(startPoint, newTurnPoint, turnPoint, islandCenter, islandRadius);
    }
    else
        complexTravelTime += calcTravelTime(startPoint, turnPoint);


    waypoints<<turnPoint[0]<<"\t"<<turnPoint[1]<<"\n";
#ifdef PRINT_WAYPOINTS
    std::cout<<turnPoint[0]<<"\t"<<turnPoint[1]<<"\n";
#endif
    complexTravelTime += calcTurnTime(startPoint, turnPoint, destPoint);

    if(intersectLineAndCircle(turnPoint, destPoint, islandCenter, islandRadius))
    {
        optimizeTurn(newTurnPoint, turnPoint, destPoint, 0x7ff0000000000000, islandCenter, islandRadius);
        processRestrictions(turnPoint, newTurnPoint, destPoint, islandCenter, islandRadius);
    }
    else
        complexTravelTime += calcTravelTime(turnPoint, destPoint);

    return complexTravelTime;
}

// Генерация скрипта для Gnuplot
void writeGnuplotScript(double* startPoint, double* destPoint, double* islandCenter, double islandRadius)
{
    double scale = calcDistance(startPoint, destPoint);
    double xmin = fmin2(startPoint[0],destPoint[0]) - scale;
    double xmax = fmax2(startPoint[0],destPoint[0]) + scale;
    double ymin = fmin2(startPoint[1],destPoint[1]) - scale;
    double ymax = fmax2(startPoint[1],destPoint[1]) + scale;
    double ratio = (ymax - ymin) / (xmax - xmin);

    std::ofstream script;
    script.open("results.plt", std::ios::out);
    //script<<"set term x11\n";
    script<<"set multiplot\n";
    script<<"set parametric\n";
    script<<"set xrange ["<<xmin<<":"<<xmax<<"]\n";
    script<<"set yrange ["<<ymin<<":"<<ymax<<"]\n";
    script<<"set size ratio "<<ratio<<"\n";
    script<<"set title \"Optimal course\"\n";
    script<<"set arrow from "<<xmax - scale / 2 + scale / 4 * world->getCosFromDegrees(90 - windAngle)<<","<<ymax - scale / 2 + scale / 4 * world->getSinFromDegrees(90 - windAngle)<<" to "<<xmax - scale / 2<<","<<ymax - scale / 2<<"\n";
    script<<"plot [0:2*pi] "<<islandRadius<<"*sin(t)+("<<islandCenter[0]<<"),"<<islandRadius<<"*cos(t)+("<<islandCenter[1]<<") lt 1 notitle\n";
    script<<"plot '"<<WAYPOINTS_FILE<<"' w l lt 3 notitle\n";
    script<<"pause mouse\n";

    script.close();
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    if(argc < 7 || (argc > 7 && argc < 10))
    {
        std::cout<<"Usage: "<<argv[0]<<" BOAT_AZIMUTH WIND_METEO_ANGLE START_X START_Y DESTINATION_X DESTINATION_Y [ISLAND_CENTER_X ISLAND_CENTER_Y ISLAND_RADIUS]";
        exit(0);
    }

    int azimuth = atoi(argv[1]);
    windAngle = atoi(argv[2]);
    double startPoint[2];
    startPoint[0] = atof(argv[3]);
    startPoint[1] = atof(argv[4]);
    double destPoint[2];
    destPoint[0] = atof(argv[5]);
    destPoint[1] = atof(argv[6]);

    double islandCenter[2];
    double islandRadius;
    if(argc > 7)
    {
        islandCenter[0] = atof(argv[7]);
        islandCenter[1] = atof(argv[8]);
        islandRadius = atof(argv[9]);
    }
    else
    {
        islandCenter[0] = 0x7ff0000000000000;
        islandCenter[1] = 0x7ff0000000000000;
        islandRadius = 1;
    }

    double travelTime = 0x7ff0000000000000;
    double turnPoint[2];
    world = new Nature();
    boat = new Boat(mass, sailSurface, s, azimuth, world);

    std::cout<<std::fixed<<std::setprecision(2);

    // Проверка, не лежат ли точки отправления и назначения в запретной области
    if(calcCircleRestriction(startPoint, islandCenter, islandRadius) <= 0)
    {
        std::cout<<"Start point is on restricted square!\n";
        exit(1);
    }
    if(calcCircleRestriction(destPoint, islandCenter, islandRadius) <= 0)
    {
        std::cout<<"Destination point is on restricted square!\n";
        exit(1);
    }

    // Вычисление времени прямого пути, если он возможен
    if(!tooSharpCourse(startPoint, destPoint) && !intersectLineAndCircle(startPoint, destPoint, islandCenter, islandRadius))
    {
        travelTime = calcTravelTime(startPoint, destPoint);

        std::cout<<"Parameters of straight course\n";
        std::cout<<"Distance: "<<calcDistance(startPoint, destPoint)<<" m\n";
        std::cout<<"Optimal sail angle: "<<boat->getSailAngle()<<" deg\n";
        std::cout<<"Max. velocity: "<<boat->getBoatVelocity()<<" m/s\n";
        std::cout<<"Estimated best travel time: "<<travelTime<<" s\n";
    }
    else
        std::cout<<"Straight course is to sharp or would lead through the island.\n";

    // Получение курса с одним поворотом
    std::cout<<"\nCalculating complex optimal course...\n";
    optimizeTurn(turnPoint, startPoint, destPoint, travelTime, islandCenter, islandRadius);


    // Обработка обхода препятствий и сохранение точек поворота в файл
    waypoints.open(WAYPOINTS_FILE, std::ios::out);
    waypoints<<std::fixed<<std::setprecision(2);
    waypoints<<startPoint[0]<<"\t"<<startPoint[1]<<"\n";
#ifdef PRINT_WAYPOINTS
    std::cout<<startPoint[0]<<"\t"<<startPoint[1]<<"\n";
#endif
    double complexTravelTime = processRestrictions(startPoint, turnPoint, destPoint, islandCenter, islandRadius);   // Обход препятствия
    waypoints<<destPoint[0]<<"\t"<<destPoint[1]<<"\n";
#ifdef PRINT_WAYPOINTS
    std::cout<<destPoint[0]<<"\t"<<destPoint[1]<<"\n";
#endif
    waypoints.close();

    writeGnuplotScript(startPoint, destPoint, islandCenter, islandRadius);

    if(!tooSharpCourse(startPoint, destPoint) &&
       !intersectLineAndCircle(startPoint, destPoint, islandCenter, islandRadius) &&
       calcTravelTime(startPoint, destPoint) < complexTravelTime)
    {
        std::cout<<"Straight course is optimal."<<"\n";
        waypoints.open(WAYPOINTS_FILE, std::ios::out);
        waypoints<<std::fixed<<std::setprecision(2);
        waypoints<<startPoint[0]<<"\t"<<startPoint[1]<<"\n";
        waypoints<<destPoint[0]<<"\t"<<destPoint[1]<<"\n";
    }
    else
    {
        std::cout<<"Best travel time for complex optimal course is ";
        std::cout<<complexTravelTime<<" s\n";
    }
    std::cout<<"Optimal waypoints were saved to \'"<<WAYPOINTS_FILE<<"\'\n";

    system("gnuplot results.plt");

    exit(0);
    return app.exec();
}
