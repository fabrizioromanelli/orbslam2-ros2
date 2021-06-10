#include "Extrapolator_Linear.hpp"

Extrapolator::Extrapolator()
{
	prevsample_T = 0;
	prevsample_S = 0;
}

Extrapolator::Extrapolator(double T, double S)
{
	prevsample_T = T;
	prevsample_S = S;
}

void Extrapolator::updateSample(double T, double S)
{
	a = (S - prevsample_S) / (T - prevsample_T);
	b = prevsample_S - a * prevsample_T;
	
	prevsample_S = S;
	prevsample_T = T;
}

double Extrapolator::get(double T)
{
	return a * T + b;
}
