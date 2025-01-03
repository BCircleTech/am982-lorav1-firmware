#include <cmath>
#include "enu.h"

ENU::ENU(const double &latitude,
         const double &longitude,
         const double &altitude)
{
    double latitudeRad = latitude * M_PI / 180;
    double longitudeRad = longitude * M_PI / 180;

    mA = 6378137;
    mF = 1 / 298.257223565;

    LLA2ECEF(latitudeRad,
             longitudeRad,
             altitude,
             mP0[0],
             mP0[1],
             mP0[2]);

    mM[0][0] = -sin(longitudeRad);
    mM[0][1] = cos(longitudeRad);
    mM[0][2] = 0;
    mM[1][0] = -sin(latitudeRad) * cos(longitudeRad);
    mM[1][1] = -sin(latitudeRad) * sin(longitudeRad);
    mM[1][2] = cos(latitudeRad);
    mM[2][0] = cos(latitudeRad) * cos(longitudeRad);
    mM[2][1] = cos(latitudeRad) * sin(longitudeRad);
    mM[2][2] = sin(latitudeRad);
}

void ENU::LLA2ENU(const double &latitude,
                  const double &longitude,
                  const double &altitude,
                  double &x,
                  double &y,
                  double &z)
{
    double latitudeRad = latitude * M_PI / 180;
    double longitudeRad = longitude * M_PI / 180;

    double xEcef;
    double yEcef;
    double zEcef;
    LLA2ECEF(latitudeRad,
             longitudeRad,
             altitude,
             xEcef,
             yEcef,
             zEcef);
    ECEF2ENU(xEcef,
             yEcef,
             zEcef,
             x,
             y,
             z);
}

void ENU::LLA2ECEF(const double &latitude,
                   const double &longitude,
                   const double &altitude,
                   double &x,
                   double &y,
                   double &z)
{
    double n = mA / sqrt(1 - mF * (2 - mF) * pow(sin(latitude), 2));
    x = (n + altitude) * cos(latitude) * cos(longitude);
    y = (n + altitude) * cos(latitude) * sin(longitude);
    z = (n * pow((1 - mF), 2) + altitude) * sin(latitude);
}

void ENU::ECEF2ENU(const double &xEcef,
                   const double &yEcef,
                   const double &zEcef,
                   double &xEnu,
                   double &yEnu,
                   double &zEnu)
{
    double deltaX = xEcef - mP0[0];
    double deltaY = yEcef - mP0[1];
    double deltaZ = zEcef - mP0[2];

    xEnu = mM[0][0] * deltaX + mM[0][1] * deltaY + mM[0][2] * deltaZ;
    yEnu = mM[1][0] * deltaX + mM[1][1] * deltaY + mM[1][2] * deltaZ;
    zEnu = mM[2][0] * deltaX + mM[2][1] * deltaY + mM[2][2] * deltaZ;
}
