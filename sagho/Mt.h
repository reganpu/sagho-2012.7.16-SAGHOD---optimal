#ifndef MT_H_
#define MT_H_

#include <vector>
#include <utility>
#include "distributions.h"
#include "traffic_source.h"

using namespace std;

class Mt
{
private:
	/*--- 引數 ---*/
	int mt_number;											//MT編號:i
	int traffic_type;										//MT的資料類型:h
	pair<int, int> network_bs_number;						//[MT所在網路編號:j (0:block, 1:GSM, 2:EDGE, 3:WCDMA, 4:LTE), MT所在BS編號]
	int car_number;											//MT所在的車子編號
	pair<double,double> car_location;						//MT所在的車子座標
	vector<vector<pair<double, double>>> bs_location;		//所有bs的座標

	queue<Packet> mt_buffer;								//packet size and packet arrival time
	static const int delay_constraint[];
	
	/*--- 需記錄的參數 ---*/
	int head_of_line_packet;
	long long num_of_dropped_packet;
	double pakcet_delay;
	long long num_of_transmitted_bit;
	long long num_of_successful_packet;

	int number_of_ho;

	/*--- 模擬須調整的參數 ---*/
	double life_time;

	/*--- 所需GSM參數 ---*/
	double g_minmum_distance;

	/*--- Edge所需參數 ---*/
	static const double EDGE_frame_size; 
	int edge_busy_frame;
	double Wi;	//the smaller Wi, the more urgent UE i. residual lifetime of head-of-line packet of UE i, which indicates the number of subframes remaining for the HOL packet not to violate QoS requirements

	/*--- 所需WCDMA參數 ---*/
	double w_minmum_distance;
	static const double WCDMA_frame_size;
	

	/*--- 所需LTE參數 ---*/
	double l_minmum_distance;
	int busy_time;				//unit: WiMAX frame; it's the time length when buffer is not empty before the current frame time
	int Vi; //the smaller Vi, the more urgent UE i. residual lifetime of head-of-line packet of UE i, which indicates the number of subframes remaining for the HOL packet not to violate QoS requirements
	/*--- bus所需參數 ---*/

public:
	Mt();
	Mt(int set_mt_number, int set_traffic_type, pair<int, int> &set_network_bs_number, int set_car_number, pair<double,double> &set_car_location, 
	   vector<vector<pair<double, double>>> set_bs_location_container);
	~Mt();

    void data_generator(const int frame);							//每單位frame的資料產生方式	
	void delay_constraint_check(const int frame, vector<vector<vector<long long> > > &);
	bool need_to_transmit(const int frame);
	bool Mt::edge_need_to_transmit(const int edge_frame);
	void update_busy_time();
	void update_edge_busy_frame();
	double bit_per_frame();
	double bit_per_edge_frame();
	double priority_value(const int frame, const vector<double> &max_bit_per_frame);
	double edge_priority_value(const int edge_frame, const vector<double> &max_bit_per_frame);

	int get_mt_number();
	int get_traffic_type();
	pair<int, int> &get_network_bs_number();
	int get_car_number();
	pair<double,double> &get_car_location();

	queue<Packet> &get_mt_buffer();

	int &get_head_of_line_packet();
	double &get_packet_delay();
	long long &get_transmitted_bits();
	long long &get_successful_packet();
	long long &get_dropped_packet();

	int &get_number_of_ho();

	double &get_life_time();

	int &get_Vi();
	double &get_Wi();

	int candidate_net_bs[3];

	double distance_between_points(const pair<double, double> &, const pair<double, double> &);	//兩點之間的距離
	double angle_between_points(const pair<double, double> &, const pair<double, double> &);	//兩點之間的夾角

};



#endif

