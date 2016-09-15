#include "Lte.h"
#include "increased_load.h"
//#include "other_functions.h"

using namespace std;

const double Lte::ISD = 0.5;
const double Lte::radius = 0.5 / (2 * cos(30 * PI/180));
const double Lte::tx_pilot_power = 46;										//46 dBm 
const double Lte::bandwidth = 3e6;											//3 MHz
const double Lte::noise_power = pow(10.0, (-174 - 30) / 10.0) * bandwidth;
const int Lte::num_of_subchannel = 15;	
const int Lte::subframe_per_frame = 10; 
const int Lte::total_num_of_RU = num_of_subchannel * subframe_per_frame;	//150 RUs
const int Lte::subcarrier_per_subchannel = 12;
const int Lte::symbol_per_subframe = 12;									//14 - 2
const int Lte::modulation_order = 6;
const int Lte::bit_per_RU = subcarrier_per_subchannel * symbol_per_subframe * modulation_order; //12*12*6 = 864 bits
const double Lte::lte_frame_size = 10e-3;

Lte::Lte(int set_bs_number, pair<double,double> &set_bs_location)
{
	bs_number = set_bs_number;									//lte bs的編號
	bs_location = set_bs_location;								//lte bs的座標
	
	load_intensity = 0;
	max_bit_per_frame.assign(2, 0.1);							//two elements, 0: http, 1: ftp; initial value = 0;
	lte_packet_delay.assign(2, 0);								//vo, vi
	lte_transmitted_bit.assign(4, 0);							//vo, vi, http, ftp	
	lte_successful_packet.assign(4, 0);							//vo, vi, http, ftp	
	lte_dropped_packet.assign(4, 0);							//vo, vi, http, ftp	


	num_of_fixed_mt = unidrnd(205, 215); //0.5(130,140) 0.9*0.8 = 0.72(180,190)
	//test
//	cout << "BS " << bs_number << endl;

	for(int i = 0; i < num_of_fixed_mt; i++)
	{
//		int h = rand()% 4;				//h = 0, 1, 2, 3          1:2:2:2
		int h = -1; 
		double temp = unifrnd(0,1);
		if(temp < 1.0 / 7)
			h = 0;
		else if(temp > 1.0 / 7)
			h = rand() % 3 + 1;

		lte_fixed_mt.push_back( Mt(i, h, make_pair(4, bs_number), 0, make_pair(bs_location.first + unifrnd(0, radius) * cos(unifrnd(0, 360) * PI / 180), bs_location.second + unifrnd(0, radius) * sin(unifrnd(0, 360) * PI / 180)), empty_bs_location) );
		//alias_of_mt_container().push_back(Mt(i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
		//test
//		cout << "i " << lte_fixed_mt[i].get_mt_number() << " h " << lte_fixed_mt[i].get_traffic_type() << " locat:(" << lte_fixed_mt[i].get_car_location().first << " " << lte_fixed_mt[i].get_car_location().second << ")" << endl;
		/*increased load*/
		load_intensity += increased_load(h, 4);
	}
	//test
//	cout << "load_intensity " << load_intensity << endl;
	//test
//	system("PAUSE");
}
Lte::~Lte(){}
/*--- 取用bs的x軸座標 ---*/
pair<double, double> Lte::get_bs_location()
{ 
	return bs_location; 
}

int Lte::get_bs_number()
{
	return bs_number;
}

vector<Mt> &Lte::get_lte_fixed_mt()
{
	return lte_fixed_mt;
}

/*--- 定義load intensity的別名 ---*/
double &Lte::get_load_intensity()
{
	return load_intensity;
}

vector<double> Lte::get_lte_packet_delay()
{
	return lte_packet_delay;
}

vector<long long> Lte::get_lte_successful_packet()
{
	return lte_successful_packet;
}

vector<long long> Lte::get_lte_transmitted_bit()
{
	return lte_transmitted_bit;
}

/*--- 每單位frame移動完的資源分配方式 ---*/
void Lte::resource_allocation(int frame, vector<vector<int>> transmittable_list, vector<Bus> &bus, vector<int> fixed_mt_trans_list)							
{
	int num_of_RU;
	int b, i;
	vector<pair<pair<int, int>, double> >::iterator v_p;
	int total_num_of_l_mt = 0;
	vector<vector<int>> resource;					//unit:RU
	int service_type;
	double elapsed_time = 0;	
	double priority_value;

	for(unsigned int i = 0; i < fixed_mt_trans_list.size(); i++)
		total_num_of_l_mt ++;

	for(unsigned int b = 0; b < transmittable_list.size(); b++)				//第幾台車
	{
		for(unsigned int i = 0; i < transmittable_list[b].size(); i++)		//第幾個人
			total_num_of_l_mt ++;	
	}

	max_bit_per_frame.assign(2, 0.1); //two elements, 0: http, 1: ftp; initial value = 0;
	//calculate priority value
	bus_mt_priority_container.clear();
	bus_mt_priority_container.reserve(total_num_of_l_mt);
	
	for(unsigned int i = 0; i < fixed_mt_trans_list.size(); i++)
	{
		//find max bit per frame for http and ftp
		service_type = lte_fixed_mt[fixed_mt_trans_list[i]].get_traffic_type();
		if (service_type == 2 || service_type == 3)
			max_bit_per_frame[service_type - 2] = max(max_bit_per_frame[service_type - 2], lte_fixed_mt[fixed_mt_trans_list[i]].bit_per_frame());

		priority_value = lte_fixed_mt[fixed_mt_trans_list[i]].priority_value(frame, max_bit_per_frame);
		//cout << "No." << bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_number() << " " << priority_value << endl;
		bus_mt_priority_container.push_back(make_pair(make_pair(0, fixed_mt_trans_list[i]), priority_value));
	}

	for(unsigned int b = 0; b < transmittable_list.size(); b++)								//第幾台車
	{
		for(unsigned int i = 0; i < transmittable_list[b].size(); i++)						//第幾個人
		{
			//find max bit per frame for http and ftp
			service_type = bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_traffic_type();
			if (service_type == 2 || service_type == 3)
				max_bit_per_frame[service_type - 2] = max(max_bit_per_frame[service_type - 2], bus[b].alias_of_mt_container()[transmittable_list[b][i]].bit_per_frame());
			
			priority_value = bus[b].alias_of_mt_container()[transmittable_list[b][i]].priority_value(frame, max_bit_per_frame);
//			cout << "No." << bus[b].alias_of_mt_container()[transmittable_list[b][i]].get_mt_number() << " " << priority_value << endl;
			bus_mt_priority_container.push_back(make_pair(make_pair(b + 1, transmittable_list[b][i]), priority_value));
		}
	}
	sort(bus_mt_priority_container.begin(), bus_mt_priority_container.end(), sort_pair);
	
// 	for(unsigned int i = 0; i < bus_mt_priority_container.size(); i++)
// 		cout << bus_mt_priority_container[i].first.second << " " ;
// 	system("PAUSE");

	//--------------------------------- priority based resource allocation --------------------------------------
	num_of_RU = total_num_of_RU;
	
	for (v_p = bus_mt_priority_container.begin(); v_p != bus_mt_priority_container.end(); v_p++)
	{
		const int Vth = 3;
		int allocated_RU = 0;
		int resource = 0;
		
		b = (*v_p).first.first;		//1, 2, ...	
		i = (*v_p).first.second;	//0 ~ 39
		
		if(b == 0)
		{
			service_type = lte_fixed_mt[i].get_traffic_type();

			if (lte_fixed_mt[i].get_Vi() >= Vth)
				allocated_RU = (int)ceil(lte_fixed_mt[i].get_head_of_line_packet() / double(lte_fixed_mt[i].get_Vi() * bit_per_RU));
			else
				allocated_RU = (int)ceil(lte_fixed_mt[i].get_head_of_line_packet() / double(bit_per_RU));

			if (allocated_RU >= num_of_RU)
				allocated_RU = num_of_RU;

			num_of_RU -= allocated_RU;
			resource = allocated_RU * bit_per_RU;

			while (frame >= lte_fixed_mt[i].get_mt_buffer().front().get_arrival_time() && resource > 0)
			{
				if (resource >= lte_fixed_mt[i].get_head_of_line_packet())
				{
					resource -= lte_fixed_mt[i].get_head_of_line_packet();
					lte_fixed_mt[i].get_successful_packet()++;
					lte_successful_packet[lte_fixed_mt[i].get_traffic_type()] ++;
					if (service_type == 0 || service_type == 1)
					{
						lte_fixed_mt[i].get_transmitted_bits() += lte_fixed_mt[i].get_mt_buffer().front().get_size();
						lte_transmitted_bit[lte_fixed_mt[i].get_traffic_type()] += lte_fixed_mt[i].get_mt_buffer().front().get_size();
						lte_fixed_mt[i].get_packet_delay() += lte_frame_size * (frame + 1 - lte_fixed_mt[i].get_mt_buffer().front().get_arrival_time());
						lte_packet_delay[lte_fixed_mt[i].get_traffic_type()] += lte_frame_size * (frame + 1 - lte_fixed_mt[i].get_mt_buffer().front().get_arrival_time());
					}
					else
					{
						lte_fixed_mt[i].get_transmitted_bits() += lte_fixed_mt[i].get_head_of_line_packet();
						lte_transmitted_bit[lte_fixed_mt[i].get_traffic_type()] += lte_fixed_mt[i].get_head_of_line_packet();
					}
					lte_fixed_mt[i].get_mt_buffer().pop();
					lte_fixed_mt[i].get_head_of_line_packet() = lte_fixed_mt[i].get_mt_buffer().front().get_size();
				}
				else
				{
					lte_fixed_mt[i].get_head_of_line_packet() -= resource;
					if (service_type == 2 || service_type == 3)
					{
						lte_fixed_mt[i].get_transmitted_bits() += resource;
						lte_transmitted_bit[lte_fixed_mt[i].get_traffic_type()] += resource;
					}
					resource = 0;
				}
			}	
		}
		else // b =1, 2, ...
		{
			service_type = bus[b - 1].alias_of_mt_container()[i].get_traffic_type();			//沒寫此會造成vector錯誤
			if (bus[b - 1].alias_of_mt_container()[i].get_Vi() >= Vth)
				allocated_RU = (int)ceil(bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() / double(bus[b - 1].alias_of_mt_container()[i].get_Vi() * bit_per_RU));
			else
				allocated_RU = (int)ceil(bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet() / double(bit_per_RU));

			if (allocated_RU >= num_of_RU)
				allocated_RU = num_of_RU;

			num_of_RU -= allocated_RU;
			resource = allocated_RU * bit_per_RU;

			while (frame >= bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time() && resource > 0)
			{
				if (resource >= bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet())
				{
					resource -= bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
					bus[b - 1].alias_of_mt_container()[i].get_successful_packet()++;
					lte_successful_packet[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] ++;
					if (service_type == 0 || service_type == 1)
					{
						bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
						lte_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] += bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
						bus[b - 1].alias_of_mt_container()[i].get_packet_delay() += lte_frame_size * (frame + 1 - bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time());
						lte_packet_delay[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] += lte_frame_size * (frame + 1 - bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time());
					}
					else
					{
						bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
						lte_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] += bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
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
						lte_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] += resource;
					}
					resource = 0;
				}
			}
		}	
	}
	/*-------------------------------------------- random allocation ----------------------------------------------------------*/
	while (bus_mt_priority_container.size() != 0 && num_of_RU > 0)
	{
		int temp = unidrnd(0, bus_mt_priority_container.size() - 1);
		b = bus_mt_priority_container[temp].first.first;
		i = bus_mt_priority_container[temp].first.second;

		num_of_RU -= 1;
		int resource = bit_per_RU;
		
		if(b == 0)
		{
			service_type = lte_fixed_mt[i].get_traffic_type();
			
			while (frame >= lte_fixed_mt[i].get_mt_buffer().front().get_arrival_time() && resource > 0)
			{
				if (resource >= lte_fixed_mt[i].get_head_of_line_packet())
				{
					resource -= lte_fixed_mt[i].get_head_of_line_packet();
					lte_fixed_mt[i].get_successful_packet() ++;
					lte_successful_packet[lte_fixed_mt[i].get_traffic_type()] ++;

					if (service_type == 0 || service_type == 1)
					{
						lte_fixed_mt[i].get_transmitted_bits() += lte_fixed_mt[i].get_mt_buffer().front().get_size();
						lte_transmitted_bit[lte_fixed_mt[i].get_traffic_type()] += lte_fixed_mt[i].get_mt_buffer().front().get_size();
						lte_fixed_mt[i].get_packet_delay() += lte_frame_size * (frame + 1 - lte_fixed_mt[i].get_mt_buffer().front().get_arrival_time());
						lte_packet_delay[lte_fixed_mt[i].get_traffic_type()] += lte_frame_size * (frame + 1 - lte_fixed_mt[i].get_mt_buffer().front().get_arrival_time());
					}
					else
					{
						lte_fixed_mt[i].get_transmitted_bits() += lte_fixed_mt[i].get_head_of_line_packet();
						lte_transmitted_bit[lte_fixed_mt[i].get_traffic_type()] += lte_fixed_mt[i].get_head_of_line_packet();
					}
					lte_fixed_mt[i].get_mt_buffer().pop();
					lte_fixed_mt[i].get_head_of_line_packet() = lte_fixed_mt[i].get_mt_buffer().front().get_size();
				}
				else
				{
					lte_fixed_mt[i].get_head_of_line_packet() -= resource;
					if (service_type == 2 || service_type == 3)
					{
						lte_fixed_mt[i].get_transmitted_bits() += resource;
						lte_transmitted_bit[lte_fixed_mt[i].get_traffic_type()] += resource;
					}
					resource = 0;
				}
			}
			if(lte_fixed_mt[i].need_to_transmit(frame) == 0)
				bus_mt_priority_container.erase(bus_mt_priority_container.begin() + temp);
		}
		else
		{
			service_type = bus[b - 1].alias_of_mt_container()[i].get_traffic_type();
			
			while (frame >= bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time() && resource > 0)
			{
				if (resource >= bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet())
				{
					resource -= bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
					bus[b - 1].alias_of_mt_container()[i].get_successful_packet() ++;
					lte_successful_packet[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] ++;

					if (service_type == 0 || service_type == 1)
					{
						bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
						lte_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] += bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_size();
						bus[b - 1].alias_of_mt_container()[i].get_packet_delay() += lte_frame_size * (frame + 1 - bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time());
						lte_packet_delay[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] += lte_frame_size * (frame + 1 - bus[b - 1].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time());
					}
					else
					{
						bus[b - 1].alias_of_mt_container()[i].get_transmitted_bits() += bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
						lte_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] += bus[b - 1].alias_of_mt_container()[i].get_head_of_line_packet();
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
						lte_transmitted_bit[bus[b - 1].alias_of_mt_container()[i].get_traffic_type()] += resource;
					}
					resource = 0;
				}
			}
			if(bus[b - 1].alias_of_mt_container()[i].need_to_transmit(frame) == 0)
				bus_mt_priority_container.erase(bus_mt_priority_container.begin() + temp);
		}
	}	
//	system("PAUSE");
}

bool Lte::sort_pair(const pair<pair<int, int>, double> &i, const pair<pair<int, int>, double> &j)
{
	return i.second > j.second; //descending order
}