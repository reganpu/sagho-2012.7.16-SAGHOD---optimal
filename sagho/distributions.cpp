//
//  distribution.cpp
//  LearningMAC
//
//  Created by Liou Yi-Shing on 12/3/2.
//  Copyright (c) 2012年 __MyCompanyName__. All rights reserved.
//

#include <cmath>
#include <cstdlib>
#include "distributions.h"

double unifrnd(double min, double max) //continuous uniform distribution, [min max]
{
	double x = (rand() * ((max - min) / double(RAND_MAX))) + min;
	return x;
}
int unidrnd(int min, int max) //discrete uniform distribution, [min, max]
{
	int x = rand() % (max - min + 1) + min;
	return x;
}
double exprnd(double mean) //exponential distribution
{
	double x, y;
	do{
		x = rand() / double(RAND_MAX);
	}while(x == 1);
	y = -mean * log(1-x);
	return y;
}
double normrnd(double mean, double std) //normal distribution
{
	double x, y, z;
	do{
		x = rand() / double(RAND_MAX);
	}while (x == 0);
	do{
		y = rand() / double(RAND_MAX);
	}while (y == 0);
	z = sqrt(-2 * log(x)) * cos(2 * PI * y) * std + mean;
	return z;
}
double lognrnd(double mean, double var) //lognormal distribution, mean: mean of lognormal, var: varience of lognormal
{
	double x, y, z;
	double mean_of_normal, std_of_normal;
	mean_of_normal = log(pow(mean, 2) / sqrt(var + pow(mean, 2)));
	std_of_normal = sqrt(log(var / pow(mean, 2) + 1));
	do{
		x = rand() / double(RAND_MAX);
	}while (x == 0);
	do{
		y = rand() / double(RAND_MAX);
	}while (y == 0);
	z = sqrt(-2 * log(x)) * cos(2 * PI * y) * std_of_normal + mean_of_normal;
	return exp(z);
}
int binornd(double p) //binomial distribution
{
	double x;
	int y;
	x = rand() / double(RAND_MAX);
	if (x <= p)
		y = 1;
	else
		y = 0;
	return y;
}
double paretornd(double index, double minimum) //pareto distribution
{
	double x, y, z; //x: uniform in (0, 1]; y: expnential with parameter index; z: pareto
	do{
		x = rand() / double(RAND_MAX);
	}while(x == 1);
	y = -(1/index) * log(1-x);
	z = minimum * exp(y);
	return z;
}

double rayleigh() //Rayleigh distribution with sigma(shape parameter形狀參數) = 1
{
	double x, y, z;
	double x1, y1,z1;
	double w;

	//standard normal distribution (mean = 0, variance =1)
	do{
		x = rand() / double(RAND_MAX);
	}while (x == 0);
	do{
		y = rand() / double(RAND_MAX);
	}while (y == 0);
	z = sqrt(-2 * log(x)) * cos(2 * PI * y) * 1 + 0;	
	//standard normal distribution (mean = 0, variance =1)
	do{
		x1 = rand() / double(RAND_MAX);
	}while (x1 == 0);
	do{
		y1 = rand() / double(RAND_MAX);
	}while (y1 == 0);
	z1 = sqrt(-2 * log(x1)) * cos(2 * PI * y1) * 1 + 0;
	
	//Rayleigh distribution
	w = sqrt(z*z + z1*z1);

	return w;
}