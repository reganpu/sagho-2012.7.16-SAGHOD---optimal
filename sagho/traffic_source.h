#ifndef _TRAFFIC_SOURCE_H_
#define _TRAFFIC_SOURCE_H_

#include <queue>

using namespace std;

class Packet
{
private:
	int size;						
	int arrival_time;										//unit: frame					
public:
	Packet(int set_size = 0, int set_arrival_time = 0)
	{
		size = set_size;
		arrival_time = set_arrival_time;
	}
	~Packet()
	{
	}
	int &get_size()
	{
		return size;
	}
	int get_arrival_time()
	{
		return arrival_time;
	}
};


void voice(queue<Packet> &, const double);
void video(queue<Packet> &, const double);
void http(queue<Packet> &, const double);
void ftp(queue<Packet> &, const double);

#endif