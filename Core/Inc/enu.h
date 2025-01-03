#ifndef ENU_H
#define ENU_H

class ENU
{
public:
    ENU(const double &latitude,
        const double &longitude,
        const double &altitude);

    void LLA2ENU(const double &latitude,
                 const double &longitude,
                 const double &altitude,
                 double &x,
                 double &y,
                 double &z);

private:
    void LLA2ECEF(const double &latitude,
                  const double &longitude,
                  const double &altitude,
                  double &x,
                  double &y,
                  double &z);

    void ECEF2ENU(const double &xEcef,
                  const double &yEcef,
                  const double &zEcef,
                  double &xEnu,
                  double &yEnu,
                  double &zEnu);

private:
    double mA;
    double mF;
    double mP0[3];

    double mM[3][3];
};

#endif