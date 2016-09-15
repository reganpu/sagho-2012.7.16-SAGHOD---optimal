#ifndef EDGE_H_
#define EDGE_H_

#include <vector>
#include <utility>
#include "Bus.h"
#include "Mt.h"
using namespace std;

class Edge
{
private:
	/*--- 引數 ---*/
	int bs_number;											//bs的編號
	pair<double,double> bs_location;						//bs的座標,原本寫 double location_x, location_y;

	/*--- 會改變值的參數 ---*/
	double load_intensity;	
	vector<double> max_bit_per_edge_frame;					//bit per frame
	vector<pair<pair<int, int>, double>> bus_mt_priority_container;

	vector<double> edge_packet_delay;
	vector<long long> edge_transmitted_bit;
	vector<long long> edge_successful_packet;
	vector<long long> edge_dropped_packet;

	/*--- 不會改變值的參數 ---*/
	static const double ISD;
	static const double radius;

	static const double tx_pilot_power; 
	static const int bit_per_slot;
	static const int allocatable_slot_per_frame[3];
	static const double edge_frame_size;	
	static const double wcdma_frame_size;
	static const double delay_constraint;			
	static const int num_of_freq_channel;	
	static const int total_num_of_slot;

	int num_of_fixed_mt;
	vector<Mt> edge_fixed_mt;
	vector<vector<pair<double, double> > > empty_bs_location;
public:
	Edge(int set_bs_number, pair<double, double> &set_bs_loaction);
	~Edge();
	pair<double,double> get_bs_location();					//取用bs的座標	
	int get_bs_number();
	double &get_load_intensity();							//定義load intensity的別名get_load_intensity()
	vector<double> get_edge_packet_delay();
	vector<long long> get_edge_successful_packet();
	vector<long long> get_edge_transmitted_bit();

	void resource_allocation(int frame, vector<vector<int>>, vector<Bus> &bus, vector<int>);					//每單位frame移動完的資源分配方式
	static bool sort_pair(const pair<pair<int, int>, double> &i, const pair<pair<int, int>, double> &j);   

	vector<Mt> &get_edge_fixed_mt();

	/*--- 存資料的容器 ---*/
	vector<vector<Mt>> bus_container;                       //lte bs裡用來儲存車子的容器
};

#endif 