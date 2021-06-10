#ifndef __EXTRAPOLATOR_H__
#define __EXTRAPOLATOR_H__

#include <math.h>

class Extrapolator
{
private:
    double tau;
    double Ss0;
    double Ss1;
    double lastabsT;

    double invTMtx[3][3];

    double samplerecvd = 0;

    double a;
    double b;
    double c;

public:
    Extrapolator(double T);
    double get(double T);
    void updateSample(double T, double S);
    void reset(void);
}

#endif
