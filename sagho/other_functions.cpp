#include "other_functions.h"

bool sort_pair(const pair<pair<int, int>, double> &i, const pair<pair<int, int>, double> &j)
{
	return i.second > j.second; //descending order
}

bool sort_pair2(const pair<double, pair<int, int> > &i, const pair<double, pair<int, int> > &j)
{
	return i.first < j.first;	//ascending order
}

bool compare(pair<int, int> &i, pair<int, int> &j)
{
	if(i.first == j.first && i.second == j.second)
		return true;
	else
		return false;
}