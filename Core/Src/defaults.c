
#include "defaults.h"
#include <math.h>

double SpeedOut(double P, double adiobata)
{
return sqrt(2*adiobata/(adiobata - 1)*Gas_R*Tout*(1-pow(1 / P, (adiobata-1)/adiobata)));	
}

// Выход в кубометрах/с
double VOut(double S, double P, double adiobata)
{
	return S * SpeedOut(P, adiobata);
}

double MOut(double S, double P, double adiobata, double Ro)
{
	double Po = 1.0;
	return S*sqrt(2*adiobata/(adiobata-1)*P*Ro*(pow(Po/P, 2/adiobata) - pow(Po/P, (adiobata+1)/adiobata) ));
}

