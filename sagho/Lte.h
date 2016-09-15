#ifndef LTE_H_
#define LTE_H_

#include <vector>
#include <algorithm>
#include <utility>
#include <cmath>
#include "Bus.h"
#include "Mt.h"
using namespace std;



class Lte
{
private:
	/*--- 引數 ---*/
	int bs_number;											//lte bs的編號
	pair<double,double> bs_location;						//lte bs的座標	
									
	
	/*--- 不會改變值的參數 ---*/
	static const double ISD;
	static const double radius;

	static const double tx_pilot_power;						//46 dBm
	static const double tx_power;
	static const double noise_power;
	static const double bandwidth;
	static const double Eb_N0[4];							//required Eb/N0 
	static const double mean_data_rate[4];
	static const int num_of_subchannel, subcarrier_per_subchannel, symbol_per_subframe, total_num_of_RU, subframe_per_frame;
	static const int modulation_order;
	static const int bit_per_RU;
	static const double lte_frame_size;

	/*--- 會改變值的參數 ---*/
	double load_intensity;	
	vector<double> max_bit_per_frame; //bit per frame
	vector<pair<pair<int, int>, double>> bus_mt_priority_container;
	
	vector<double> lte_packet_delay;
	vector<long long> lte_transmitted_bit;
	vector<long long> lte_successful_packet;
	vector<long long> lte_dropped_packet;
	
	int num_of_fixed_mt;
	vector<Mt> lte_fixed_mt;
	vector<vector<pair<double, double> > > empty_bs_location;
public:
	Lte(int set_bs_number, pair<double, double> &set_bs_location);
	~Lte();
	pair<double, double> get_bs_location();					//取用bs的座標
	int get_bs_number();
	double &get_load_intensity();							//定義load intensity的別名get_load_intensity()
	vector<double> get_lte_packet_delay();
	vector<long long> get_lte_successful_packet();
	vector<long long> get_lte_transmitted_bit();

	vector<Mt> &get_lte_fixed_mt();

	void resource_allocation(int frame, vector<vector<int>>, vector<Bus> &bus, vector<int>);					//每單位frame移動完的資源分配方式
	static bool sort_pair(const pair<pair<int, int>, double> &i, const pair<pair<int, int>, double> &j);   //why不用static會錯?

	/*--- 存資料的容器 ---*/
	vector<vector<Mt>> bus_container;                       //lte bs裡用來儲存車子的容器
};

#endif