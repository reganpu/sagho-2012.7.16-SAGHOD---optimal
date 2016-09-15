#ifndef _SIMULATED_ANNEALING_SCHEME_H_
#define _SIMULATED_ANNEALING_SCHEME_H_

#include <vector>
#include <iostream>
#include <algorithm>
#include "Mt.h"
#include "increased_load.h"
#include "Bus.h"
#include <cmath>

using namespace std;

void sa_scheme(Bus &, vector<vector<int>> &,int frame, double &, double &, double &, double &, 
			   double[], vector<vector<pair<double, double>>> , vector<vector<int>> , int [],
	           double [], double [],
	           double &);

#endif 