#ifndef SA_SCHEME_H_
#define SA_SCHEME_H_

#include <vector>
#include <iostream>
#include <algorithm>
#include "Mt.h"
#include "increased_load.h"
#include "Bus.h"
#include <cmath>
#include "other_functions.h"
// #include <fstream>	//ofstream
// #include <string>
// #include <sstream>  //ostringstream

using namespace std;

void sa_scheme(Bus &, vector<vector<int>> &,int frame, double[], 
			   double &, double &, double &, double &, 
			   vector<vector<double> > &, vector<vector<double> > &, vector<vector<double> > &,
			   int &);


#endif 

