#ifndef GSM_H_
#define GSM_H_

#include <vector>
#include <utility>
#include "Bus.h"
#include "Mt.h"
using namespace std;

class Gsm
{
private:
	/*--- 引數 ---*/
	int bs_number;											//lte bs 的編號
	pair<double,double> bs_location;						//lte bs 的座標							

	/*--- 會改變值的參數 ---*/
	double load_intensity;		
	vector<pair<pair<int, int>, double>> bus_mt_priority_container;

	double gsm_packet_delay;
	long long gsm_transmitted_bit;
	long long gsm_successful_packet;
	long long gsm_dropped_packet;

	/*--- 不會改變值的參數 ---*/
	static const double ISD;
	static const double radius;

	static const double tx_pilot_power; 
	static const int bit_per_slot;
	static const int allocatable_slot_per_frame;
	static const double gsm_frame_size;
	static const double wcdma_frame_size;
	static const double delay_constraint;					//40 ms
	static const int num_of_freq_channel;
	static const int num_of_t_channel;

	int num_of_fixed_mt;
	vector<Mt> gsm_fixed_mt;
	vector<vector<pair<double, double> > > empty_bs_location;
public:
	Gsm(int set_bs_number, pair<double, double> &set_bs_loaction);
	~Gsm();
	
	pair<double,double> get_bs_location();					//取用bs的座標	
	int get_bs_number();
	double &get_load_intensity();							//定義load intensity的別名get_load_intensity()
	double get_gsm_packet_delay();
	long long get_gsm_successful_packet();
	long long get_gsm_transmitted_bit();

	vector<Mt> &get_gsm_fixed_mt();

	void resource_allocation(int frame, vector<vector<int>>, vector<Bus> &bus, vector<int> );					//每單位frame移動完的資源分配方式

	/*--- 存資料的容器 ---*/
	vector<vector<Mt>> bus_container;                      //lte bs裡用來儲存車子的容器
};

#endif
