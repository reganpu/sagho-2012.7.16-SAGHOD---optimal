#ifndef WCDMA_H_
#define WCDMA_H_

#include <vector>
#include <utility>
#include <cmath>
#include "Bus.h"
#include "Mt.h"

using namespace std;

class Wcdma
{
private:
	/*--- 引數 ---*/
	int bs_number;											//wcdma bs的編號
	pair<double,double> bs_location;						//wcdma bs的座標,原本寫 double location_x, location_y;						
	
	/*--- 不會改變值的參數 ---*/
	static const double ISD;
	static const double radius;

	static const int num_of_wcdma_bs;						//wcdma bs的數量
	static const double num_of_frame;						//模擬時間
	static const double wcdma_frame_size;
	static const double chip_rate;								
//	static const double cell_radius;						//細胞半徑
	static const double other_to_own_interference_ratio;	
	static const double Eb_N0[4];							//required Eb/N0 	 
	static const double mean_data_rate[4];
	static const double tx_pilot_power;     
	static const double min_distance_between_bs_and_mt;
	
	static const double tx_power;
	static const double bandwidth;
	static const double noise_power;

	/*--- 會改變值的參數 ---*/
	double load_intensity;	
	double achievable_transmission_rate[4];                    
	double rx_power, SINR;
	vector<double> wcdma_packet_delay;
	vector<long long> wcdma_transmitted_bit;
	vector<long long> wcdma_successful_packet;
	vector<long long> wcdma_dropped_packet;

	int num_of_fixed_mt;
	vector<Mt> wcdma_fixed_mt;
	vector<vector<pair<double, double> > > empty_bs_location;
public:
	Wcdma(int set_bs_number, pair<double, double> &set_bs_loaction);		//建構子
	~Wcdma();
//	double get_cell_radius();                               //取用細胞半徑
	pair<double, double> get_bs_location();					//取用bs座標位置	
	int get_bs_number();
	double &get_load_intensity();							//定義load intensity的別名get_load_intensity()
	vector<double> get_wcdma_packet_delay();
	vector<long long> get_wcdma_successful_packet();
	vector<long long> get_wcdma_transmitted_bit();

	vector<Mt> &get_wcdma_fixed_mt();

	void resource_allocation(int frame, vector<vector<int>>, vector<Bus> &bus, vector<int>);

	/*--- 存資料的容器 ---*/
	vector<vector<Mt>> mt_container;						//wcdma bs裡用來儲存某些車子的的某些不同traffic的MT的容器

};

#endif 