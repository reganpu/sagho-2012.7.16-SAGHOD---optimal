#include <cmath>
#include <vector>
#include <utility>
#include "Edge.h"
#include "increased_load.h"
using namespace std;

const double Edge::ISD = 1.0;
const double Edge::radius = 1.0 / (2 * cos(30 * PI/180));

const double Edge::tx_pilot_power = 31;						//31 dBm
const int Edge::bit_per_slot = 114 * 3;						//342 bit / 10 ms = 74.1 kbps 無干擾下的位元速率
const int Edge::allocatable_slot_per_frame[3] = {1, 2, 3};	//74.1 kbps 148.2 kbps 222.3 kbps
const double Edge::edge_frame_size = 4.615e-3;	
const double Edge::wcdma_frame_size = 10e-3;
const double Edge::delay_constraint = 100e-3;				//100 ms
const int Edge::num_of_freq_channel = 18;					//frequency reuse factor = 3
const int Edge::total_num_of_slot = num_of_freq_channel * 3;

/*--- 建構子 ---*/
Edge::Edge(int set_bs_number, pair<double,double> &set_bs_location)
{
	bs_number = set_bs_number;								//bs的編號
	bs_location = set_bs_location;							//bs的座標
	load_intensity = 0;

	edge_packet_delay.assign(1, 0);							//vi
	edge_transmitted_bit.assign(3, 0);						//0vi, 1http, 2ftp	
	edge_successful_packet.assign(3, 0);					//vi, http, ftp	
	edge_dropped_packet.assign(3, 0);						//vi, http, ftp	

	num_of_fixed_mt = unidrnd(64, 65);	//0.9*0.8=0.72(45,55)
	//test
//	cout << "BS " << bs_number << endl;

	for(int i = 0; i < num_of_fixed_mt; i++)
	{
		double temp = unifrnd(0, 1);
		int h = -1;				//h = 1, 2, 3    2:2:1
		if(temp < 1.0 / 5)
			h = 3;
		else
			h = rand() % 2 + 1;			//h = 1, 2, 3
		edge_fixed_mt.push_back( Mt(i, h, make_pair(2, bs_number), 0, make_pair(bs_location.first + unifrnd(0, radius) * cos(unifrnd(0, 360) * PI / 180), bs_location.second + unifrnd(0, radius) * sin(unifrnd(0, 360) * PI / 180)), empty_bs_location) );
		//alias_of_mt_container().push_back(Mt(i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
		//test
//		cout << "i " << edge_fixed_mt[i].get_mt_number() << " h " << edge_fixed_mt[i].get_traffic_type() << " locat:(" << edge_fixed_mt[i].get_car_location().first << " " << edge_fixed_mt[i].get_car_location().second << ")" << endl;
		/*increased load*/
		load_intensity += increased_load(h, 2);
		
	}
	//test
//	cout << "load_intensity " << load_intensity << endl;
	//test
//	system("PAUSE");
}
Edge::~Edge(){}
/*--- 取用bs的座標 ---*/

pair<double, double> Edge::get_bs_location()
{
	return bs_location;
}

int Edge::get_bs_number()
{
	return bs_number;
}

vector<Mt> &Edge::get_edge_fixed_mt()
{
	return edge_fixed_mt;
}

/*--- 定義load intensity的別名 ---*/
double &Edge::get_load_intensity()
{
	return load_intensity;
}

vector<double> Edge::get_edge_packet_delay()
{
	return edge_packet_delay;
}

vector<long long> Edge::get_edge_successful_packet()
{
	return edge_successful_packet;
}

vector<long long> Edge::get_edge_transmitted_bit()
{
	return edge_transmitted_bit;
}

/*--- 每單位frame移動完的資源分配方式 ---*/
void Edge::resource_allocation(int edge_frame, vector<vector<int>> transmittable_list, vector<Bus> &bus, vector<int> fixed_mt_trans_list)							
{
	int num_of_slot;
	int b, i;
	vector<pair<pair<int, int>, double> >::iterator v_p;
	int service_type;
	int total_num_of_e_mt = 0;
	double edge_priority_value;

	for(unsigned int i = 0; i < fixed_mt_trans_list.size(); i++)
		total_num_of_e_mt ++;

	for(unsigned int b = 0; b < transmittable_list.size(); b++)				//第幾台車
	{
		for(unsigned int i = 0; i < transmittable_list[b].size(); i++)		//第幾個人
			total_num_of_e_mt ++;	
	}

	max_bit_per_edge_frame.assign(2, 0.1); //two elements, 0: http, 1: ftp; initial value = 0;
	//calculate priority value
	bus_mt_priority_container.clear();
	bus_mt_priority_container.reserve(total_num_of_e_mt);

	for(unsigned int i = 0; i < fixed_mt_trans_list.size(); i++)
	{
		//find max bit per edge frame for http and ftp
		service_type = edge_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type();

		if (service_type == 2 || service_type == 3)
			max_bit_per_edge_frame[service_type - 2] = max(max_bit_per_edge_frame[service_type - 2], edge_fixed_mt[fixed_mt_trans_list[i]].bit_per_edge_frame());
		edge_priority_value = edge_fixed_mt[fixed_mt_trans_list[i]].edge_priority_value(edge_frame, max_bit_per_edge_frame);
	
		bus_mt_priority_container.push_back(make_pair(make_pair(0, fixed_mt_trans_list[i]), edge_priority_value));	
	}

	for(unsigned int b = 0; b < transmittable_list.size(); b++)								//第幾台車
	{
		for(unsigned int i = 0; i < transmittable_list[b].size(); i++)						//第幾個人
		{
			//find max bit per edge frame for http and ftp
			service_type = bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type();
			
			
			if (service_type == 2 || service_type == 3)
				max_bit_per_edge_frame[service_type - 2] = max(max_bit_per_edge_frame[service_type - 2], bus[b].alias_of_mt_container()[transmittable_list[b][i]].bit_per_edge_frame());
			edge_priority_value = bus[b].alias_of_mt_container()[transmittable_list[b][i]].edge_priority_value(edge_frame, max_bit_per_edge_frame);
// 		    cout << "No." << bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_number() 
// 				 << " h " << bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type()
// 				 << " priority_value " << edge_priority_value << endl;
// 			if(bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type() == 1)
// 				edge_priority_value = 3;
// 			if(bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type() == 2 || bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type() == 3)
// 				edge_priority_value = 2;
// 			else
// 				edge_priority_value = 1;		//decision寫好不讓voice進入就不用寫
			bus_mt_priority_container.push_back(make_pair(make_pair(b + 1, transmittable_list[b][i]), edge_priority_value));
 		}
	}

	sort(bus_mt_priority_container.begin(), bus_mt_priority_container.end(), sort_pair);

// 	//debug
// 	if(edge_frame == 16561)		//12 29 38
// 	{
// 		for(unsigned int i = 0; i < bus_mt_priority_container.size(); i++)
// 		cout << bus_mt_priority_container[i].first.second << " " ;
// 		system("PAUSE");
// 	}
	
// 	for(unsigned int i = 0; i < bus_mt_priority_container.size(); i++)
// 	 	cout << bus_mt_priority_container[i].first.second << " " ;
// 	system("PAUSE");

	num_of_slot = total_num_of_slot;

	for (v_p = bus_mt_priority_container.begin(); v_p != bus_mt_priority_container.end(); v_p++)
	{
		double Wth = 3 * 4.615e-3;
		int allocated_slot = 0;
		int resource = 0;
		
		b = (*v_p).first.first;		//"0", 1, 2, ...	
		i = (*v_p).first.second;	//0 ~ 39, 0 ~ fixed_mt_trans_list.size() 
		
		if(b == 0)
		{
			service_type = edge_fixed_mt[i].get_traffic_type();
			if (edge_fixed_mt[i].get_Wi() >= Wth)
				allocated_slot = min((int)ceil(edge_fixed_mt[i].get_head_of_line_packet() / double(edge_fixed_mt[i].get_Wi() * bit_per_slot)), 3);
			else
				allocated_slot = min((int)ceil(edge_fixed_mt[i].get_head_of_line_packet() / double(bit_per_slot)), 3);
			
			num_of_slot -= allocated_slot;
			resource = allocated_slot * bit_per_slot;

			while ((edge_frame * edge_frame_size) >= (edge_fixed_mt[i].get_mt_buffer().front().get_arrival_time() * wcdma_frame_size) && resource > 0)
			{
				if (resource >= edge_fixed_mt[i].get_head_of_line_packet())
				{
					resource -= edge_fixed_mt[i].get_head_of_line_packet();	
					edge_fixed_mt[i].get_successful_packet()++;
					edge_successful_packet[edge_fixed_mt[i].get_traffic_type() - 1]++;

					if (service_type == 1)
					{
						edge_fixed_mt[i].get_transmitted_bits() += edge_fixed_mt[i].get_mt_buffer().front().get_size();
						edge_transmitted_bit[edge_fixed_mt[i].get_traffic_type() - 1] += edge_fixed_mt[i].get_mt_buffer().front().get_size();
						edge_fixed_mt[i].get_packet_delay() += edge_frame_size * (edge_frame + 1) - wcdma_frame_size * edge_fixed_mt[i].get_mt_buffer().front().get_arrival_time();
						edge_packet_delay[edge_fixed_mt[i].get_traffic_type() - 1] += edge_frame_size * (edge_frame + 1) - wcdma_frame_size * edge_fixed_mt[i].get_mt_buffer().front().get_arrival_time();
					}
					else if((service_type == 2)||(service_type == 3))
					{
						edge_fixed_mt[i].get_transmitted_bits() += edge_fixed_mt[i].get_head_of_line_packet();
						edge_transmitted_bit[edge_fixed_mt[i].get_traffic_type() - 1] += edge_fixed_mt[i].get_head_of_line_packet();
					}
					edge_fixed_mt[i].get_mt_buffer().pop();
					edge_fixed_mt[i].get_head_of_line_packet() = edge_fixed_mt[i].get_mt_buffer().front().get_size();		
				}
				else
				{
					edge_fixed_mt[i].get_head_of_line_packet() -= resource;
					if (service_type == 2 || service_type == 3)
					{
						edge_fixed_mt[i].get_transmitted_bits() += resource;
						edge_transmitted_bit[edge_fixed_mt[i].get_traffic_type() - 1] += resource;

					}
					resource = 0;
				}
			}

		}
		else //b = 1,2,...
		{
			service_type = bus[b - 1].alias_of_mt_container()[i].get_traffic_type();			//沒寫此會造成vector錯誤
			if (bus[b - 1].alias_of_mt_container()[i].get_Wi() >= Wth)
				allocated_slot = min((int)ceil(bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() / double(bus[b - 1].alias_of_mt_container()[i].get_Wi() * bit_per_slot)), 3);
			else
				allocated_slot = min((int)ceil(bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() / double(bit_per_slot)), 3);

			num_of_slot -= allocated_slot;
			resource = allocated_slot * bit_per_slot;

			while ((edge_frame * edge_frame_size) >= (bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time() * wcdma_frame_size) && resource > 0)
			{
				// 			//debug
				// 			if(edge_frame == 16561)
				// 			{
				// 				cout << "edge_frame " << edge_frame * edge_frame_size << endl;
				// 				cout << "arrival time " << bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time() * wcdma_frame_size << endl;
				// 				cout << "resource " << resource << endl;	//342
				// 				cout << "HOL_packet " << bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() << endl;	//110 
				// 				system("PAUSE");
				// 			}

				if (resource >= bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet())
				{
					resource -= bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();	
					bus[b - 1].alias_of_mt_container()[i].get_successful_packet()++;
					edge_successful_packet[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1]++;

					if (service_type == 1)
					{
						bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
						edge_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1] += bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
						bus[b - 1].alias_of_mt_container()[i].get_packet_delay() += edge_frame_size * (edge_frame + 1) - wcdma_frame_size * bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time();
						edge_packet_delay[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1] += edge_frame_size * (edge_frame + 1) - wcdma_frame_size * bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time();
					}
					else if((service_type == 2)||(service_type == 3))
					{
						bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
						edge_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1] += bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
					}
					bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().pop();
					bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() = bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();		
				}
				else
				{
					bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() -= resource;
					if (service_type == 2 || service_type == 3)
					{
						bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += resource;
						edge_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1] += resource;

					}
					resource = 0;
				}
			}
		}	
	}

// 	while (bus_mt_priority_container.size() != 0 && num_of_slot > 0)
// 	{
// 		int temp = unidrnd(0, bus_mt_priority_container.size() - 1);
// 		b = bus_mt_priority_container[temp].first.first;
// 		i = bus_mt_priority_container[temp].first.second;
// 
// 		num_of_slot -= 1;
// 		int resource = bit_per_slot;
// 		service_type = bus[b - 1].alias_of_mt_container()[i].get_traffic_type();	
// 
// 		while ((edge_frame * edge_frame_size) >= (bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time() * wcdma_frame_size) && resource > 0)
// 		{
// 			if (resource >= bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet())
// 			{
// 				resource -= bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
// 				bus[b - 1].alias_of_mt_container()[i].get_successful_packet() ++;
// 				edge_successful_packet[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] ++;
// 
// 				if (service_type == 1)
// 				{
// 					bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
// 					edge_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1] += bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
// 					bus[b - 1].alias_of_mt_container()[i].get_packet_delay() += edge_frame_size * (edge_frame + 1) - wcdma_frame_size * bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time();
// 					edge_packet_delay[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1] += edge_frame_size * (edge_frame + 1) - wcdma_frame_size * bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time();
// 				}
// 				else if((service_type == 2)||(service_type == 3))
// 				{
// 					bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
// 					edge_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1] += bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
// 				}
// 				bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().pop();
// 				bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() = bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
// 			}
// 			else
// 			{
// 				bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() -= resource;
// 				if (service_type == 2 || service_type == 3)
// 				{
// 					bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += resource;
// 					edge_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type() - 1] += resource;
// 				}
// 				resource = 0;
// 			}
// 		}
// 		if(bus[b - 1].alias_of_mt_container()[i].need_to_transmit(edge_frame) == 0)
// 			bus_mt_priority_container.erase(bus_mt_priority_container.begin() + temp);
// 
// 	}
}

bool Edge::sort_pair(const pair<pair<int, int>, double> &i, const pair<pair<int, int>, double> &j)
{
	return i.second > j.second; //descending order
}