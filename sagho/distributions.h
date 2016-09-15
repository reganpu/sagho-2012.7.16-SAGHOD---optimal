//
//  distributions.h
//  LearningMAC
//
//  Created by Liou Yi-Shing on 12/3/2.
//  Copyright (c) 2012¦~ __MyCompanyName__. All rights reserved.
//

#ifndef DISTRIBUTIONS_H_
#define DISTRIBUTIONS_H_

const double PI = 3.14159265358979323846;

double unifrnd(double, double);
int unidrnd(int, int);
double exprnd(double);
double normrnd(double, double);
double lognrnd(double, double);
int binornd(double);
double paretornd(double, double);

double rayleigh();

#endif