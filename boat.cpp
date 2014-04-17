#include "boat.h"

public Boat::Boat (double m1, double sailSurf, double s1, double azimuth1, Nature N)
{
    Nature = N;

    m = m1;
    SailS = sailSurf;
    s = s1;
    Azimuth = azimuth1;
    VBoat = 0;
    a = 0;

    x = 36.0;
    y = 55.0;

    Sail2BoatAngle = 0;
    Sail2BoatOrintation = "0";

    UpdateSailOrintation();

    RudderAngle = 0;



    Array cx  = new double [37];
    cx.SetValue (0.1, 0);
    cx.SetValue( 0.1, 1);
    cx.SetValue(0.15, 2);
    cx.SetValue(0.18, 3);
    cx.SetValue(0.23, 4);
    cx.SetValue(0.3, 5);
    cx.SetValue(0.4, 6);
    cx.SetValue(0.6, 7);
    cx.SetValue(0.7, 8);
    cx.SetValue(0.8, 9);
    cx.SetValue(0.85, 10);
    cx.SetValue(0.93, 11);
    cx.SetValue(1, 12);
    cx.SetValue(1.07, 13);
    cx.SetValue(1.12, 14);
    cx.SetValue(1.19, 15);
    cx.SetValue(1.2, 16);
    cx.SetValue(1.23, 17);
    cx.SetValue(1.25, 18);
    cx.SetValue(1.23, 19);
    cx.SetValue(1.2, 20);
    cx.SetValue(1.19, 21);
    cx.SetValue(1.12, 22);
    cx.SetValue(1.07, 23);
    cx.SetValue(1, 24);
    cx.SetValue(0.93, 25);
    cx.SetValue(0.85, 26);
    cx.SetValue(0.8, 27);
    cx.SetValue(0.7, 28);
    cx.SetValue(0.6, 29);
    cx.SetValue(0.4, 30);
    cx.SetValue(0.3, 31);
    cx.SetValue(0.23, 32);
    cx.SetValue(0.18, 33);
    cx.SetValue(0.15, 34);
    cx.SetValue(0.1, 35);
    cx.SetValue(0.1, 36);

    int Size = cx.Length;
    Cx = new Interpolation (cx, Size, 5, 0, 180);

    Array cy = new double [37];
    cy.SetValue(0, 0);
    cy.SetValue(0.3, 1);
    cy.SetValue(0.5, 2);
    cy.SetValue(0.9, 3);
    cy.SetValue(1.2, 4);
    cy.SetValue(1.4, 5);
    cy.SetValue(1.6, 6);
    cy.SetValue(1.3, 7);
    cy.SetValue(1.14, 8);
    cy.SetValue(1.09, 9);
    cy.SetValue(1, 10);
    cy.SetValue(0.87, 11);
    cy.SetValue(0.78, 12);
    cy.SetValue(0.66, 13);
    cy.SetValue(0.55, 14);
    cy.SetValue(0.4, 15);
    cy.SetValue(0.3, 16);
    cy.SetValue(0.2, 17);
    cy.SetValue(0, 18);
    cy.SetValue(0.3, 19);
    cy.SetValue(0.35, 20);
    cy.SetValue(0.4, 21);
    cy.SetValue(0.45, 22);
    cy.SetValue(0.5, 23);
    cy.SetValue(0.6, 24);
    cy.SetValue(0.8, 25);
    cy.SetValue(1.0, 26);
    cy.SetValue(1.2, 27);
    cy.SetValue(1.1, 28);
    cy.SetValue(0.9, 29);
    cy.SetValue(0.8, 30);
    cy.SetValue(0.7, 31);
    cy.SetValue(0.4, 32);
    cy.SetValue(0.35, 33);
    cy.SetValue(0.2, 34);
    cy.SetValue(0.1, 35);
    cy.SetValue(0, 36);

    Size = cy.Length;
    Cy = new Interpolation (cy, Size, 5, 0, 180);

}


// /////////////////////////////////////////////////////////////////////
public double getM ()  {return m;}
public double getSailS () {return SailS;}
public double getS ()  {return s;}
public double getAzimuth () {return Azimuth;}
public double getX () {return x;}
public double getY() {return y;}
public double getVBoat() { return VBoat;}
public double getA() {return a;}
public double getSailAngle() { return Sail2BoatAngle; }
public double getFFriction() { return FFriction; }
public double getFTraction() { return FTraction; }
public double getFWaterResist() { return FWaterResist; }

public string getSailSide() { return SailSide; }
public double getAttacAngle() { return AttacAngle; }
public string getSail2BoatOrintation() { return Sail2BoatOrintation; }

public double getRudderAngle() { return RudderAngle; }


public void setSailS(double s) { SailS = s; }
public void setS(double s1) { s = s1; }
public void setM(double m1) { m = m1; }

public void setAzimuth (double azimuth) { Azimuth = azimuth; }
public void setVBoat (double v) {VBoat = v;}
public void setA (double a1) {a = a1;}

public void setSail2BoatAngle (double angle) {Sail2BoatAngle = angle;}
public void setSail2BoatOrintation(string orintation) { Sail2BoatOrintation = orintation; }

public void setX(double x1) {x = x1;}
public void setY(double y1) {y = y1;}
public void setRudderAngle(double angle) { RudderAngle = angle; }


public double DecRudderAngle()
{
    if (RudderAngle > -30) --RudderAngle;
    return RudderAngle;
}

public double IncRudderAngle()
{
    if (RudderAngle < 30) ++RudderAngle;
    return RudderAngle;
}

public void OptimiseSailAngle (int WindAngle)
{
    double speed = 0;
    Azimuth = 0;
    Nature.setWindAngle(WindAngle);

    Sail2BoatOrintation = "L";
    Sail2BoatAngle = 0;
    double nextSpeed = BoatSpeed();
    do
    {
        Sail2BoatAngle += 5;
        speed = nextSpeed;
        nextSpeed = BoatSpeed();
    } while (speed <= nextSpeed && Sail2BoatAngle < 90);
    Sail2BoatAngle -= 5;
    double LeftMax = Sail2BoatAngle;
    double LeftVMax = speed;

    Sail2BoatOrintation = "R";
    Sail2BoatAngle = 0;
    nextSpeed = BoatSpeed();
    do
    {
        Sail2BoatAngle += 5;
        speed = nextSpeed;
        nextSpeed = BoatSpeed();
    } while (speed <= nextSpeed && Sail2BoatAngle < 90);
    Sail2BoatAngle -= 5;
    double RightMax = Sail2BoatAngle;
    double RightVMax = speed;

    if (LeftVMax >= RightVMax)
    {
        Sail2BoatAngle = LeftMax;
        Sail2BoatOrintation = "L";
        VBoat = LeftVMax;
    }
    else
    {
        Sail2BoatAngle = RightMax;
        Sail2BoatOrintation = "R";
        VBoat = RightVMax;
    }


}

public double BoatSpeed ()
{
    double delta_a = 0.001;
    VBoat = 0;
    a = 0;

    do
    {
        Step ();
    } while (a > delta_a);

    return VBoat;
}

// ///////////////////////////////////////////////////////////////////////////////////

public double SailAzimuth()
{
    if (Sail2BoatOrintation == "L") return Nature.add360(Azimuth, Sail2BoatAngle);
    else return Nature.sub360(Azimuth, Sail2BoatAngle);
}

// /////////////////////////////////////////////////////////////////////////////////

public void UpdateSailOrintation()
{
    AttacAngle = Nature.sub360(SailAzimuth(), Nature.getWindAngle(VBoat, Azimuth));

    if (AttacAngle > 180)
    {
        SailSide = "Out";
        AttacAngle = 360 - AttacAngle;
    }
    else SailSide = "In";

    if (Sail2BoatOrintation == "L")
    {
        if (SailSide == "In") SailSide = "Out";
        else SailSide = "In";
    }
}

// //////////////////////////////////////////////////////////////////////////////////

public void CountForses()
{
    FWaterResist = -Nature.FWaterResist(VBoat, s);
    if (VBoat < 0) FWaterResist *= -1;

    FFriction = Nature.FFriction(Cx.get(AttacAngle), SailS, Nature.getVWind(VBoat, Azimuth));
    FTraction = Nature.FTraction(Cy.get(AttacAngle), SailS, Nature.getVWind(VBoat, Azimuth));
    if (getSailSide() == "Out") FTraction *= -1;
}

//
/////////////////////////////////////////////////////////////////////////////////////////
public void Step()
{
    ChangeAzimuth();
    UpdateSailOrintation();

    CountForses();

    a = (
        Nature.SinDeg(Sail2BoatAngle) * FTraction
        - FFriction * Nature.CosDeg(Nature.getWindAngleToBoat(Azimuth))
        + FWaterResist
        ) / m;


    double delta_t = 0.001;
    x += (VBoat * delta_t + a * delta_t * delta_t / 2) * Nature.SinDeg(Azimuth) / 100000.0;
    y -= (VBoat * delta_t + a * delta_t * delta_t / 2) * Nature.CosDeg(Azimuth) / 100000.0;

    VBoat = VBoat + a * delta_t;

}

// ////////////////////////////////////////////////////////////////////////////////////

public void ChangeAzimuth()
{
    double c = 2000;
    double delta_azimuth = VBoat / 100000.0 * RudderAngle * c;
    Azimuth = Nature.add360(Azimuth, delta_azimuth);

}

public void TurnSailR() { TurnSail("R"); }
public void TurnSailL() { TurnSail("L"); }

void TurnSail(string dir)
{
    string opdir = "L";
    if (dir == "L") opdir = "R";

    if (Sail2BoatOrintation == dir && Sail2BoatAngle < 90)
        ++Sail2BoatAngle;

    else if (Sail2BoatOrintation == "0")
    {
        Sail2BoatAngle = 1;
        Sail2BoatOrintation = dir;
    }

    else if (Sail2BoatOrintation == opdir)
    {
        --Sail2BoatAngle;
        if (Sail2BoatAngle == 0) Sail2BoatOrintation = "0";
    }

}
