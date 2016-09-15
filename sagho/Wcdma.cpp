#include "Wcdma.h"
#include "increased_load.h"

using namespace std;

const double Wcdma::ISD = 0.645;
const double Wcdma::radius = 0.645 / (2 * cos(30 * PI/180));

const int Wcdma::num_of_wcdma_bs = 5; 
const double Wcdma::num_of_frame = 100e3;
const double Wcdma::wcdma_frame_size = 10e-3;
const double Wcdma::chip_rate = 3.84e6;
//const double Wcdma::cell_radius = 0.3225*2/sqrt(3.0);
const double Wcdma::other_to_own_interference_ratio = 0.55;													  //Macro cell with omnidirectional antennas 55%
const double Wcdma::Eb_N0[4] = {pow(10.0, 4.0/10), pow(10.0, 3.0/10), pow(10.0, 2.0/10), pow(10.0, 1.5/10)};  //required Eb/N0: vo:4dB,vi:3dB, http:2dB, ftp:1.5dB
const double Wcdma::mean_data_rate[4] = {11.2e3 * (1.0/2.35), 64e3, 16.14e3, 88.9e3};						  //vo:4.77 kbps, vi:64 kbps, *http:16.14 kbps*, ftp:88.9 kbps
const double Wcdma::tx_pilot_power = 33;																	  //33 dBm  
const double Wcdma::min_distance_between_bs_and_mt = 0.056;													  //56 m = 0.056 km

const double Wcdma::tx_power = 0.25;  //watt
const double Wcdma::bandwidth = 5e6;
const double Wcdma::noise_power = pow(10.0, (-174 - 30) / 10.0) * bandwidth;


/*--- 建構子 ---*/
Wcdma::Wcdma(int set_bs_number, pair<double,double> &set_bs_location)
{
	bs_number = set_bs_number;								//wcdma bs的編號
	bs_location = set_bs_location;							//wcdma bs的座標
	
	load_intensity = 0; 

	for(int i = 0; i < 4; i++)
		achievable_transmission_rate[i] = 0;

	rx_power = -1;
	SINR = -1;

	wcdma_packet_delay.assign(2, 0);				//vo, vi
	wcdma_transmitted_bit.assign(4, 0);				//vo, vi, http, ftp	
	wcdma_successful_packet.assign(4, 0);			//vo, vi, http, ftp	
	wcdma_dropped_packet.assign(4, 0);				//vo, vi, http, ftp	


	num_of_fixed_mt = unidrnd(40, 45); //0.7*0.8 = 0.56(30,40)
	//test
//	cout << "BS " << bs_number << endl;

	for(int i = 0; i < num_of_fixed_mt; i++)
	{
//		int h = rand()% 4;				//h = 0, 1, 2, 3
		int h = -1;
		double temp = unifrnd(0, 1);
		if(temp < 6.0 / 11)
			h = 0;
		else if(temp > 6.0 / 11 && temp < 8.0 / 11)
			h = 1;
		else if(temp > 8.0 / 11 && temp < 10.0 / 11)
			h = 2;
		else if(temp > 10.0 /11)
			h = 3;
		wcdma_fixed_mt.push_back( Mt(i, h, make_pair(3, bs_number), 0, make_pair(bs_location.first + unifrnd(0, radius) * cos(unifrnd(0, 360) * PI / 180), bs_location.second + unifrnd(0, radius) * sin(unifrnd(0, 360) * PI / 180)), empty_bs_location) );
		//		alias_of_mt_container().push_back(Mt(i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
		//test
//		cout << "i " << wcdma_fixed_mt[i].get_mt_number() << " h " << wcdma_fixed_mt[i].get_traffic_type() << " locat:(" << wcdma_fixed_mt[i].get_car_location().first << " " << wcdma_fixed_mt[i].get_car_location().second << ")" << endl;
		/*increased load*/
		load_intensity += increased_load(h, 3);
	}
	//test
//	cout << "load_intensity " << load_intensity << endl;
	//test
//	system("PAUSE");

}
Wcdma::~Wcdma(){}

/*--- 取用bs的座標 ---*/
pair<double, double> Wcdma::get_bs_location()
{
	return bs_location;
}

int Wcdma::get_bs_number()
{
	return bs_number;
}

vector<Mt> &Wcdma::get_wcdma_fixed_mt()
{
	return wcdma_fixed_mt;
}

/*--- 定義load intensity的別名 ---*/
double &Wcdma::get_load_intensity()
{
	return load_intensity;
}

vector<double> Wcdma::get_wcdma_packet_delay()
{
	return wcdma_packet_delay;
}

vector<long long> Wcdma::get_wcdma_successful_packet()
{
	return wcdma_successful_packet;
}

vector<long long> Wcdma::get_wcdma_transmitted_bit()
{
	return wcdma_transmitted_bit;
}

/*--- 每單位frame的資源分配方式 () ---*/
void Wcdma::resource_allocation(int frame, vector<vector<int>> transmittable_list, vector<Bus> &bus, vector<int> fixed_mt_trans_list)
{
	int total_num_of_w_mt = 0;						//有資料要傳且serving bs為wcdma的總人數
	int resource[4] = {-1, -1, -1, -1};				
	double elapsed_time = 0;						

	for(unsigned int i = 0; i < fixed_mt_trans_list.size(); i++)
		total_num_of_w_mt ++;

	for(unsigned int b = 0; b < transmittable_list.size(); b++)				//第幾台車
	{
		for(unsigned int i = 0; i < transmittable_list[b].size(); i++)		//第幾個人
		{
			total_num_of_w_mt ++;	
			/***************************************************************************************/
// 			Mt mt = bus[b].alias_of_mt_container()[transmittable_list[b][i]];
// 			cout << "mt No." << mt.get_mt_number() << " " 
// 				 << " h: " << mt.get_traffic_type() << " "
// 				 << " j: " << mt.get_network_bs_number().first << " "
// 				 << " buf size: " << mt.get_mt_buffer().size() << " "
// 			 	 << " front.size: " << mt.get_mt_buffer().front().get_size() << " " 
// 				 << " front.arri time: " << mt.get_mt_buffer().front().get_arrival_time() << endl; 
			/***************************************************************************************/
		}
	}
	/*************************************************************************/
//  	cout << "total wcdma transmittable list: " << total_num_of_w_mt << endl;
//  	system("PAUSE");
	/*************************************************************************/

	rx_power = (total_num_of_w_mt * tx_power + noise_power) * (1 + other_to_own_interference_ratio);
	SINR = tx_power / (rx_power - tx_power);
	
	for(int h = 0; h < 4; h++)															//vo, vi, http, ftp
	{
		achievable_transmission_rate[h] = chip_rate / Eb_N0[h] * SINR;					//bps
		resource[h] = (int)floor(achievable_transmission_rate[h] * wcdma_frame_size);	//bit		
	}
	/****************************************************/
// 	cout << "achievable_transmission_rate:" << endl;
// 	for(int h = 0; h < 4; h++)
// 		cout << achievable_transmission_rate[h] << " " ;
// 	system("PAUSE");
	/****************************************************/

	for(unsigned int i = 0; i < fixed_mt_trans_list.size(); i++)
	{
		int resource_copy[4] = {resource[0], resource[1], resource[2], resource[3]};

		while(frame >= wcdma_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_arrival_time() && resource_copy[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()] > 0 ) 
		{
			if(resource_copy[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()] >= wcdma_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet())
			{
				resource_copy[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()] -= wcdma_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet();
				wcdma_fixed_mt[fixed_mt_trans_list[i]].get_successful_packet()++;
				wcdma_successful_packet[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()]++;
				
				if(wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type() == 0 || wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type() == 1)
				{
					//傳送的bit數增加 
					wcdma_fixed_mt[fixed_mt_trans_list[i]].get_transmitted_bits() += wcdma_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_size();
					wcdma_transmitted_bit[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()] += wcdma_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_size();
					//在一個frame內有在服務的時間
					elapsed_time += wcdma_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet() / achievable_transmission_rate[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()];
					//total packet delay增加
					wcdma_fixed_mt[fixed_mt_trans_list[i]].get_packet_delay() += wcdma_frame_size * (frame - wcdma_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_arrival_time()) + elapsed_time;
					wcdma_packet_delay[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()] += wcdma_frame_size * (frame - wcdma_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_arrival_time()) + elapsed_time;

				}
				else
				{
					//傳送的bit數增加
					wcdma_fixed_mt[fixed_mt_trans_list[i]].get_transmitted_bits() += wcdma_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet(); 
				}
				//服務完HOL packet,清除HOL packet
				wcdma_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().pop();
				//修改HOL packet內容成下一個HOL packet bit數 
				wcdma_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet() = wcdma_fixed_mt[fixed_mt_trans_list[i]].get_mt_buffer().front().get_size();
			}
			else
			{
				//計算HOL packet剩餘須被服務的bit數
				wcdma_fixed_mt[fixed_mt_trans_list[i]].get_head_of_line_packet() -= resource_copy[ wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type() ];
				//http, ftp
				if(wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type() == 2 || wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type() == 3)
				{
					//傳送的bit數增加(只算http, ftp. RT的要整個packet傳送完再算才有意義)
					wcdma_fixed_mt[fixed_mt_trans_list[i]].get_transmitted_bits() += resource_copy[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()];
					wcdma_transmitted_bit[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()] += resource_copy[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()];
				}
				//資源分光了所以歸零
				resource_copy[wcdma_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type()] = 0;
			}
		}
	}


	for(unsigned int b = 0; b < transmittable_list.size(); b++)								//第幾台車
	{
		for(unsigned int i = 0; i < transmittable_list[b].size(); i++)						//第幾個人
		{
			Mt mt = bus[b].alias_of_mt_container()[transmittable_list[b][i]];

			int resource_copy[4] = {resource[0], resource[1], resource[2], resource[3]};
				
			//當目前模擬時間大於HOL packet的到達時間(mt有資料要傳) 且 網路有資源可服務
			while(frame >= bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_arrival_time() && resource_copy[mt.get_traffic_type()] > 0 ) 
			{
			/******************************************************************************************************************/
// 			cout << "mt No." << mt.get_mt_number() << " " 
// 				 << " h: " << mt.get_traffic_type() << " "
// 				 << " j: " << mt.get_network_bs_number().first << " "
// 			     << " buf size: " << mt.get_mt_buffer().size() << " "
// 			     << " fro.size: " << mt.get_mt_buffer().front().get_size() << " " 
// 			     << " HOL: " << mt.get_head_of_line_packet() << " "
// 			     << " fro.a_t : " << mt.get_mt_buffer().front().get_arrival_time() << endl ; 
// 			cout << "current frame: " << frame << endl;
// 			cout << "resource: " << resource_copy[mt.get_traffic_type()] << endl;
					/******************************************************************************************************************/

				//若網路資源比HOL packet多
   				if(resource_copy[mt.get_traffic_type()] >= bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet())
				{ 
					//減去HOL packet大小的剩餘網路資源(for all traffic type)
					resource_copy[bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type()] -= bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet();
					//成功傳送的packet數增加(for all traffic type)
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_successful_packet()++;
					wcdma_successful_packet[mt.get_traffic_type()]++;
						
					/******************************************************************************/
// 					cout << "remaining resource: " << resource_copy[mt.get_traffic_type()] << endl;
// 					cout << "success pkt: "<< mt.get_successful_packet() << endl;
					/******************************************************************************/

					//voice, video
					if(mt.get_traffic_type() == 0 || mt.get_traffic_type() == 1)
					{
						//傳送的bit數增加 
						bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_transmitted_bits() += bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_size();
						wcdma_transmitted_bit[mt.get_traffic_type()] += bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_size();
						//在一個frame內有在服務的時間
						elapsed_time += bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet() / achievable_transmission_rate[bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type()];
						//total packet delay增加
						bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_packet_delay() += wcdma_frame_size * (frame - bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_arrival_time()) + elapsed_time;
						wcdma_packet_delay[mt.get_traffic_type()] += wcdma_frame_size * (frame - bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_arrival_time()) + elapsed_time;
					}
					//http, ftp 
					else
					{
						//傳送的bit數增加
						bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_transmitted_bits() += bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet(); 
					}
					//服務完HOL packet,清除HOL packet
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().pop();
					//修改HOL packet內容成下一個HOL packet bit數 
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet() = bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_buffer().front().get_size();

					/*****************************************************************************************/
//					cout << "next HOL arrival time: " << mt.get_mt_buffer().front().get_arrival_time() << endl;
					/*****************************************************************************************/
 				}
				//若網路資源比HOL packet少
				else
				{
					//計算HOL packet剩餘須被服務的bit數
					bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_head_of_line_packet() -= resource_copy[ bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type() ];
					//http, ftp
					if(mt.get_traffic_type() == 2 || mt.get_traffic_type() == 3)
					{
						//傳送的bit數增加(只算http, ftp. RT的要整個packet傳送完再算才有意義)
						bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_transmitted_bits() += resource_copy[bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type()];
						wcdma_transmitted_bit[mt.get_traffic_type()] += resource_copy[bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type()];
					}
					//資源分光了所以歸零
					resource_copy[mt.get_traffic_type()] = 0;

					/******************************************************************************/
// 					cout << "remaining resource: " << resource_copy[mt.get_traffic_type()] << endl;
// 					cout << "success pkt: "<< mt.get_successful_packet() << endl;
					/******************************************************************************/
				}
			}		        
		} 
	}
}
