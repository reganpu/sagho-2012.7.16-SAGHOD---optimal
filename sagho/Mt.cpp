#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include "Mt.h"
#include "path_loss.h"
#include "distributions.h"
#include "traffic_source.h"

using namespace std;

const double Mt::WCDMA_frame_size = 10e-3;	//10ms
const double Mt::EDGE_frame_size = 4.615e-3; 
const int Mt::delay_constraint[] = {int(40e-3 / WCDMA_frame_size), int(100e-3 / WCDMA_frame_size)};	//4 frame; 10 frame

extern vector<vector<vector<long long>>> dropped_packet;

/*--- 預設建構子 ---*/
Mt::Mt()
{
	mt_number = -1;																		//MT編號
	traffic_type = -1;																	//MT的資料類型
	network_bs_number = make_pair(-1, -1);												//MT所在(網路編號,BS編號)				(會被ho_decision()不斷修改)
	car_number = -1;																	//MT所在的車子編號						(會被mobility()不斷修改)----還沒寫好傳回值       
	car_location = make_pair(-1, -1);													//MT所在的車子座標						(會被mobility()不斷修改)----還沒寫好傳回值
	bs_location.assign(3, vector<pair<double, double>> (27, make_pair(-1.0, -1.0)));	//所有bs的座標

	g_minmum_distance = 0.02;								//20 m
	w_minmum_distance = 0.015;								//15 m
	l_minmum_distance = 0.012;								//12 m; calculation in p.477

	mt_buffer.push(Packet(0, 0));							//buffer裡初始有一個size 0 arrival time 0 的 packet				(會被不斷修改)

	number_of_ho = 0;

	head_of_line_packet = 0;
	pakcet_delay = 0;
	num_of_transmitted_bit = 0;
	num_of_successful_packet = 0;
	num_of_dropped_packet = 0;

	busy_time = -1;											//minus one due to "before the current frame time"
	edge_busy_frame = -1;
	Vi = 0;
	Wi = 0;

	candidate_net_bs[0] = -1;
	candidate_net_bs[1] = -1;
	candidate_net_bs[2] = -1;

	/*--- 模擬要調整的參數(暫定平均值為模擬時間) ---*/
	life_time = exprnd(15000 * 10e-3);								//MT的通話時間; 單位:sec
}
Mt::Mt(int set_mt_number ,int set_traffic_type, pair<int, int> &set_network_bs_number, int set_car_number, pair<double,double> &set_car_location, 
	   vector<vector<pair<double, double>>> set_bs_location_container)
{
	/*--- 引數 ---*/
	mt_number = set_mt_number;								//MT編號: 0~39
	traffic_type = set_traffic_type;						//MT的資料類型: 0~3
	network_bs_number = set_network_bs_number;				//MT所在(網路編號:1~4,BS編號:1~7,19,27)				(會被ho_decision()不斷修改)
	car_number = set_car_number;							//MT所在的車子編號						(會被mobility()不斷修改)----還沒寫好傳回值       
	car_location = set_car_location;						//MT所在的車子座標						(會被mobility()不斷修改)----還沒寫好傳回值
	bs_location = set_bs_location_container;				//所有bs的座標

	g_minmum_distance = 0.02;								//20 m
	w_minmum_distance = 0.015;								//15 m
	l_minmum_distance = 0.012;								//12 m; calculation in p.477

	mt_buffer.push(Packet(0, 0));							//buffer裡初始有一個size 0 arrival time 0 的 packet	(buffer會被不斷修改)

	number_of_ho = 0;

	head_of_line_packet = 0;
	pakcet_delay = 0;
	num_of_transmitted_bit = 0;
	num_of_successful_packet = 0;
	num_of_dropped_packet = 0;

	busy_time = -1;											//minus one due to "before the current frame time"
	edge_busy_frame = -1;
	Vi = 0;
	Wi = 0;

	candidate_net_bs[0] = -1;
	candidate_net_bs[1] = -1;
	candidate_net_bs[2] = -1;

	/*--- 模擬要調整的參數(暫定平均值為模擬時間) ---*/
	life_time = exprnd(15000 * 10e-3);								//MT的通話時間; 單位:frame
}
Mt::~Mt()
{

}
/*--- 取用MT編號 ---*/
int Mt::get_mt_number()
{
	return mt_number;
}
/*--- 取用MT資料類型 ---*/
int Mt::get_traffic_type()
{
	return traffic_type;
}
/*--- 取用MT的(serving network, serving BS) ---*/
pair<int, int> &Mt::get_network_bs_number()
{
	return network_bs_number;
}

int Mt::get_car_number()
{
	return car_number;
}

pair<double,double> &Mt::get_car_location()
{
	return car_location;
}

queue<Packet> &Mt::get_mt_buffer()
{
	 return mt_buffer;
}

int &Mt::get_head_of_line_packet()
{
	return head_of_line_packet;
}

/*--- 取用QoS參數 ---*/
double &Mt::get_packet_delay()
{
	return pakcet_delay;
}
long long &Mt::get_transmitted_bits()
{
	return num_of_transmitted_bit;
}
long long &Mt::get_successful_packet()
{
	return num_of_successful_packet;
}
long long &Mt::get_dropped_packet()
{
	return num_of_dropped_packet;
}

double &Mt::get_life_time()
{
	return life_time;
}

int &Mt::get_number_of_ho()
{
	return number_of_ho;
}

/*--- 產生資料 ---*/
void Mt::data_generator(const int frame)
{
	//當模擬時間大於等於buffer裡最後一個packet的arrival time
	while (frame >= mt_buffer.back().get_arrival_time())		//mt_buffer.back().get_arrival_time()初始為0    ">="
	{
		switch (traffic_type)
		{
		case 0:
			voice(mt_buffer, WCDMA_frame_size);
			break;
		case 1:
			video(mt_buffer, WCDMA_frame_size);
			break;
		case 2:
			http(mt_buffer, WCDMA_frame_size);
			break;
		case 3:
			ftp(mt_buffer, WCDMA_frame_size);
			break;
		default:
			cout << "service type is wrong" << endl;
			break;
		}
	}
	//更新HOL packet
	if (head_of_line_packet == 0)
		head_of_line_packet = mt_buffer.front().get_size();
}

/*--- voice和video產生的資料超過delay限制就丟掉 ---*/
void Mt::delay_constraint_check(const int frame, vector<vector<vector<long long> > > &dropped_pkt)
{
	if (traffic_type == 0 || traffic_type == 1)
	{
		while (frame - mt_buffer.front().get_arrival_time() >= delay_constraint[traffic_type])
		{
			num_of_dropped_packet ++;
			//network(0(network 1),1(network 2),2(network 3),3(network 4)), bs (0,...), traffic type(0,1) 
			dropped_pkt[network_bs_number.first - 1][network_bs_number.second - 1][traffic_type] ++;
			mt_buffer.pop();
			//更新HOL packet
			head_of_line_packet = mt_buffer.front().get_size();	
		}
	}
}

bool Mt::need_to_transmit(const int frame)
{
	if (frame >= mt_buffer.front().get_arrival_time())
		return true;
	else
		return false;
}



bool Mt::edge_need_to_transmit(const int edge_frame)
{
	if (edge_frame * 4.615e-3 >= mt_buffer.front().get_arrival_time() * 10e-3)
		return true;
	else
		return false;
}

/*--- Lte ---*/
void Mt::update_busy_time()
{
	if (traffic_type == 2 || traffic_type == 3)
		busy_time += 1;
}
/*--- Lte ---*/
double Mt::bit_per_frame()			//平均每個frame傳輸的bit數
{
	if (busy_time != 0)
		return num_of_transmitted_bit / double(busy_time);
	else
		return 0;
}

int &Mt::get_Vi()
{
	return Vi;
}
/*--- Lte ---*/
double Mt::priority_value(const int frame, const vector<double> &max_bit_per_frame)
{
	int ai[4] = {3, 3, 2, 1};	//default priority constant for UE i
	double ui;					//priority value for UE i 

	if (traffic_type == 0 || traffic_type == 1)
		Vi = delay_constraint[traffic_type] - (frame - mt_buffer.front().get_arrival_time());    //delay requirement - packet delay
	else
		Vi = (int)floor((head_of_line_packet + num_of_transmitted_bit) / max_bit_per_frame[traffic_type - 2] - busy_time); 
	if (Vi > 0)
	{
		int head_of_line_packet_delay = frame - mt_buffer.front().get_arrival_time();				//frame

		ui = (1 + head_of_line_packet_delay / double(Vi + head_of_line_packet_delay)) * ai[traffic_type];
	}
	else
	{
		ui = 2.0 * ai[traffic_type];
	}
	return ui;
}

/*--- Edge ---*/
void Mt::update_edge_busy_frame()
{
	if (traffic_type == 2 || traffic_type == 3)
		edge_busy_frame += 2;
}
/*--- Edge ---*/
double Mt::bit_per_edge_frame()
{
	if (edge_busy_frame != 0)
		return num_of_transmitted_bit / double(edge_busy_frame);
	else
		return 0;
}

double &Mt::get_Wi()
{
	return Wi;
}

/*--- Edge ---*/
double Mt::edge_priority_value(const int edge_frame, const vector<double> &max_bit_per_edge_frame)
{
	double ai[3] = {3, 2, 1};		//default priority constant for UE i
	double ui;						//priority value for UE i 

	if (traffic_type == 1)
		Wi = delay_constraint[traffic_type] * WCDMA_frame_size - (edge_frame * EDGE_frame_size - mt_buffer.front().get_arrival_time() * WCDMA_frame_size);    //delay requirement - packet delay
	else if(traffic_type == 2 ||traffic_type == 3)
		Wi = (head_of_line_packet + num_of_transmitted_bit) / (max_bit_per_edge_frame[traffic_type - 2] / 4.615e-3)  - edge_busy_frame * EDGE_frame_size; 
	if (Wi > 0)
	{
		double head_of_line_packet_delay = edge_frame * EDGE_frame_size - mt_buffer.front().get_arrival_time() * WCDMA_frame_size;	//sec

		ui = ( 1.0 + (head_of_line_packet_delay / (Wi + head_of_line_packet_delay)) ) * ai[traffic_type];
	}
	else
	{
		ui = 2.0 * ai[traffic_type];
	}
	return ui;
}

/*--- 兩點之間的距離 ---*/
double Mt::distance_between_points(const pair<double, double> &i, const pair<double, double> &j)
{
	return hypot(i.first - j.first, i.second - j.second);
}

/*--- 兩點之間的夾角 ---*/
double Mt::angle_between_points(const pair<double, double> &i, const pair<double, double> &j)		//j -> i
{
	double delta_x, delta_y, angle;

	delta_x = i.first - j.first;
	delta_y = i.second - j.second;
	if (delta_x >= 0 && delta_y >= 0)																//first quadrant
	{
		if (delta_x != 0)
			angle = atan(delta_y / delta_x);
		else if (delta_y == 0)					//(0, 0)
			angle = 0;
		else
			angle = PI / 2;
	}
	else if (delta_x < 0 && delta_y >= 0)															//second quadrant
	{
		angle = PI - atan(abs(delta_y / delta_x));
	}
	else if (delta_x <= 0 && delta_y < 0)															//third quadrant
	{
		if (delta_x != 0)
			angle = PI + atan(abs(delta_y / delta_x));
		else
			angle = (3.0 / 2) * PI;
	}
	else																							//fourth quadrant
	{
		angle = 2 * PI - atan(abs(delta_y / delta_x));
	}
	return angle;
}