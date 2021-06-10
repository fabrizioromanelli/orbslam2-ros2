#ifndef __EXTRAPOLATOR_H__
#define __EXTRAPOLATOR_H__

class Extrapolator
{
	private:
		double prevsample_T;
		double prevsample_S;
		double a;
		double b;
		
	public:
		Extrapolator();
		Extrapolator(double T, double S);
		double get(double T);
		void updateSample(double T, double S);
};

#endif

