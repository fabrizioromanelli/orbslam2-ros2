#include "Extrapolator_Quadratic_FixedTime.hpp"

Extrapolator::Extrapolator(double T)
{
	tau = T;
	Ss0 = 0;
	Ss1 = 0;
	
	samplerecvd = 0;
	
	invTMtx[0][0] = 1 / (2 * tau * tau);
	invTMtx[0][1] = - 1 / (tau * tau);
	invTMtx[0][2] = 1/ (2 * tau * tau);
	
	invTMtx[1][0] = - 5 / (2 * tau);
	invTMtx[1][1] = 4 / tau;
	invTMtx[1][2] = - 3 / (2 * tau);
	
	invTMtx[2][0] = 3;
	invTMtx[2][1] = -3;
	invTMtx[2][2] = 1;
	
	a = 0;
	b = 0;
	c = 0;
}

void Extrapolator::updateSample(double T, double S)
{
	if(samplerecvd == 0) {
		Ss0 = S;
		samplerecvd++;
	}
	else if(samplerecvd == 1) {
		Ss1 = S;
		samplerecvd++;
	}
	else {
		a = invTMtx[0][0] * Ss0 + invTMtx[0][1] * Ss1 + invTMtx[0][2] * S;
		b = invTMtx[1][0] * Ss0 + invTMtx[1][1] * Ss1 + invTMtx[1][2] * S;
		c = invTMtx[2][0] * Ss0 + invTMtx[2][1] * Ss1 + invTMtx[2][2] * S;
		
		lastabsT = T;
		
		Ss0 = Ss1;
		Ss1 = S;
	}
}

double Extrapolator::get(double T)
{
	double t = T - lastabsT;

	return a * t * t + b * t + c;
}
