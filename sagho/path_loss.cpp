#include <iostream>
#include <cmath>
#include "path_loss.h"
#include "distributions.h"

using namespace std;

/*--- GSM/EDGE path loss ---*/ //Tx power: 31dBm
double gsm_edge_path_loss(double d)
{
	double pl = 0.0;				
	double output = 0.0;

	pl = 129.1 + 34*log10(d);
//	pl = 129.1 + 34*log10(d) + 10*log10( rayleigh() );
//	pl = 129.1 + 34*log10(d) + normrnd(0.0, 5.0) + 10*log10( rayleigh() );		//standard deviation: 5dB

	output = pl;
	return output;
}

/*--- WCDMA path loss ---*/   //Tx power: 33dBm
double wcdma_path_loss(double d)
{

	double pl = 0.0;				
	double output = 0.0;

	pl = 138.1 + 35.2*log10(d);
//	pl = 138.1 + 35.2*log10(d) + 10*log10( rayleigh() );
//	pl = 138.1 + 35.2*log10(d) + normrnd(0.0, 6.0) + 10*log10( rayleigh() );		//standard deviation: 6dB

	output = pl;
	return output;
}//p.212 p.222

/*--- LTE path loss ---*/   //Tx power: 46dBm
double lte_path_loss(double d)
{

	double pl;
	double output;

	pl = 156.28 + 37.6*log10(d);
//	pl = 156.28 + 37.6*log10(d) + 10*log10( rayleigh() );
//	pl = 156.28 + 37.6*log10(d) + normrnd(0.0, 8.0) + 10*log10( rayleigh() );	//standard deviation: 8dB

	output = pl;
	return output;
}//p.476


//假設接收訊號強度:-40dBm 為最好,可推得與bs的最近距離

//31 + 40 = 71dB  = 129.1 + 34*log10(d)       -> d ~ 0.02 km
//33 + 40 = 73dB  = 138.1 + 35.2*log10(d)		-> d ~ 0.015 km
//46 + 40 = 84dB  = 156.28 + 37.6*log10(d)	-> d = 0.012 km


double path_loss(int j, double d)
{
	double pl;
	double output;

// 	if(j == 1 || j == 2)
// 	{	pl = 129.1 + 34*log10(d) + 10*log10( rayleigh() );}
// 	else if(j == 3)
// 	{	pl = 138.1 + 35.2*log10(d) + 10*log10( rayleigh() );}
// 	else if(j == 4)
// 	{	pl = 156.28 + 37.6*log10(d) + 10*log10( rayleigh() );}
// 	else
// 	{
// 		cout << "error" << endl;
// 		system("PAUSE");
// 	}
	switch(j)
	{
	case 0 :
		pl = 129.1 + 34*log10(d);
//		pl = 129.1 + 34*log10(d) + 10*log10( rayleigh() );
//		pl = 129.1 + 34*log10(d) + normrnd(0.0, 5.0) + 10*log10( rayleigh() );
		break;
	case 1:
		pl = 138.1 + 35.2*log10(d);
//		pl = 138.1 + 35.2*log10(d) + 10*log10( rayleigh() );
//		pl = 138.1 + 35.2*log10(d) + normrnd(0.0, 6.0) + 10*log10( rayleigh() );	
		break;
	case 2:
		pl = 156.28 + 37.6*log10(d);
//		pl = 156.28 + 37.6*log10(d) + 10*log10( rayleigh() );
//		pl = 156.28 + 37.6*log10(d) + normrnd(0.0, 8.0) + 10*log10( rayleigh() );
		break;
	}
	output = pl;
	return output;
}