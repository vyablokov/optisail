#include "interpolation.h"

Interpolation::Interpolation(QVector<double> *c, int n, int s, double ll, double rl)
{
    step = s;

    leftLimit = ll;
    rightLimit = rl;

//    table = new double[n];
    table = new QVector<double>(n);

    for (int i = 0; i < n; i++)
//        Table.SetValue(c.GetValue(i), i);
        table->insert(i, c->takeAt(i));
}

double Interpolation::get(double angle)
{
    if ( ! (leftLimit <= angle && angle <= rightLimit) )
        return 0;

//    int i = Convert.ToInt32((angle - leftLimit) / step);
    int i = (int)((angle - leftLimit) / step);

    if (angle == (angle / step) * step)
    {
//        double c = Convert.ToDouble(table.GetValue(i));
        double c = table->takeAt(i);
        return c;
    }

    if (angle > 175)
        //return Convert.ToDouble(table.GetValue(36));
        return table->takeAt(36);

//    int angle1 = Convert.ToInt32(i * step);
    int angle1 = (int)(i * step);
    int angle2 = angle1 + step;
//    double c1 = Convert.ToDouble(Table.GetValue(i));
    double c1 = table->takeAt(i);
//    double c2 = Convert.ToDouble(Table.GetValue(i + 1));
    double c2 = table->takeAt(i + 1);
    return ((c2 - c1) * (angle - angle1)) / (angle2 - angle1) + c1;
}
