#include <cmath>
#include <vector>
#include <utility>
#include "Gsm.h"
#include "increased_load.h"
using namespace std;

const double Gsm::ISD = 1.0;
const double Gsm::radius = 1.0 / (2 * cos(30 * PI/180));


const double Gsm::tx_pilot_power = 31;					//31 dBm   p.90
const int Gsm::bit_per_slot = 114;						//114 bit / 4.615 ms = 24.7 kbps 無干擾下真正用來傳資料的位元速率		
const int Gsm::allocatable_slot_per_frame = 1;
const double Gsm::gsm_frame_size = 4.615e-3;	
const double Gsm::wcdma_frame_size = 10e-3;
const double Gsm::delay_constraint = 40e-3;				//40 ms
const int Gsm::num_of_freq_channel = 18;				//frequency reuse factor = 3
const int Gsm::num_of_t_channel = 3 + 7;				//兩個Tx

/*--- 建構子 ---*/
Gsm::Gsm(int set_bs_number, pair<double,double> &set_bs_location)
{
	bs_number = set_bs_number;							//bs的編號
	bs_location = set_bs_location;						//bs的座標
	load_intensity = 0;

	gsm_packet_delay = 0;								//vo
	gsm_transmitted_bit = 0;							//vo 
	gsm_successful_packet = 0;							//vo
	gsm_dropped_packet = 0;								//vo

	num_of_fixed_mt = unidrnd(155, 165);	//0.5:(90,110) 0.8(140,150)
	//test
//	cout << "BS " << bs_number << endl;

	for(int i = 0; i < num_of_fixed_mt; i++)
	{
		int h = 0;				
		gsm_fixed_mt.push_back( Mt(i, h, make_pair(1, bs_number), 0, make_pair(bs_location.first + unifrnd(0, radius) * cos(unifrnd(0, 360) * PI / 180), bs_location.second + unifrnd(0, radius) * sin(unifrnd(0, 360) * PI / 180)), empty_bs_location) );
		//alias_of_mt_container().push_back(Mt(i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
		//test
//		cout << "i " << gsm_fixed_mt[i].get_mt_number() << " h " << gsm_fixed_mt[i].get_traffic_type() << " locat:(" << gsm_fixed_mt[i].get_car_location().first << " " << gsm_fixed_mt[i].get_car_location().second << ")" << endl;
		/*increased load*/
		load_intensity += increased_load(h, 1);
	}
	//test
//	cout << "load_intensity " << load_intensity << endl;
	//test
//	system("PAUSE");
}
Gsm::~Gsm(){}
/*--- 取用bs的座標 ---*/

pair<double, double> Gsm::get_bs_location()
{
	return bs_location;
}

int Gsm::get_bs_number()
{
	return bs_number;
}

vector<Mt> &Gsm::get_gsm_fixed_mt()
{
	return gsm_fixed_mt;
}

/*--- 定義load intensity的別名 ---*/
double &Gsm::get_load_intensity()
{
	return load_intensity;
}

double Gsm::get_gsm_packet_delay()
{
	return gsm_packet_delay;
}

long long Gsm::get_gsm_successful_packet()
{
	return gsm_successful_packet;
}

long long Gsm::get_gsm_transmitted_bit()
{
	return gsm_transmitted_bit;
}

/*--- 每單位frame移動完的資源分配方式 ---*/
void Gsm::resource_allocation(int gsm_frame, vector<vector<int>> transmittable_list, vector<Bus> &bus, vector<int> fixed_mt_trans_list)							
{
	int total_num_of_g_mt = 0;
	double difference = 0.0;		// sec

	for(unsigned int i = 0; i < fixed_mt_trans_list.size(); i++)			//fixed mt
		total_num_of_g_mt++;
	
	//debug
// 	cout << "test!" << endl;
// 	cout << "total_num_of_fixed_mt " << total_num_of_g_mt <<endl;
// 	system("PAUSE");

	for(unsigned int b = 0; b < transmittable_list.size(); b++)				//第幾台車
	{
		for(unsigned int i = 0; i < transmittable_list[b].size(); i++)		//第幾個人
			total_num_of_g_mt ++;	
	}

	//debug
// 	cout << "test!" << endl;
// 	cout << "total_num_of_g_mt " << total_num_of_g_mt <<endl;
// 	system("PAUSE");

	for(unsigned int i = 0; i < fixed_mt_trans_list.size(); i++)
	{
		int resource = bit_per_slot;
		//當目前模擬時間大於HOL packet的到達時間(mt有資料要傳) 且 網路有資源可服務
		while((gsm_frame * gsm_frame_size) >= (gsm_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_arrival_time() * wcdma_frame_size) && resource > 0 )	
		{
			//若網路資源比HOL packet多
			if(resource >= gsm_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet())
			{
				resource -= gsm_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet();
				gsm_fixed_mt[fixed_mt_trans_list[i]].get_successful_packet()++;
				gsm_successful_packet++;

				gsm_fixed_mt[fixed_mt_trans_list[i]].get_transmitted_bits() += gsm_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_size();
				gsm_transmitted_bit += gsm_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_size();
				gsm_fixed_mt[fixed_mt_trans_list[i]].get_packet_delay() += gsm_frame_size * (gsm_frame + 1) - wcdma_frame_size * gsm_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_arrival_time();
				gsm_packet_delay += gsm_frame_size * (gsm_frame + 1) - wcdma_frame_size * gsm_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_arrival_time();

				gsm_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().pop();
				gsm_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet() = gsm_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_size();		
			}
			//若網路資源比HOL packet少
			else
			{
				gsm_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet() -= resource;

				resource = 0;
			}
		}
	}

	//debug
// 	cout << "test!!!!" << endl;
// 	system("PAUSE");

	//不需要分priority,每個人都有自己的專屬slot,照著迴圈一個一個人分即可
	for(unsigned int b = 0; b < transmittable_list.size(); b++)								//第幾台車
	{
		for(unsigned int i = 0; i < transmittable_list[b].size(); i++)						//第幾個人
		{
			int resource = bit_per_slot;
			//當目前模擬時間大於HOL packet的到達時間(mt有資料要傳) 且 網路有資源可服務
			while ((gsm_frame * gsm_frame_size)  >= (bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_arrival_time() * wcdma_frame_size) && resource > 0)
			{
				//若網路資源比HOL packet多
				if(resource >= bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet())
				{
					resource -= bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet();
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_successful_packet()++;
					gsm_successful_packet ++;
					
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_transmitted_bits() += bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_size();
					gsm_transmitted_bit += bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_size();
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_packet_delay() += gsm_frame_size * (gsm_frame + 1) - wcdma_frame_size * bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_arrival_time();
					gsm_packet_delay += gsm_frame_size * (gsm_frame + 1) - wcdma_frame_size * bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_arrival_time();
				
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().pop();
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet() = bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_size();		//出錯因為寫成 bus[b].alias_of_mt_container()[i]
				}
				//若網路資源比HOL packet少
				else
				{
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet() -= resource;
					
					resource = 0;
				}
			}
		}
	}

}