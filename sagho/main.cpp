#include <iostream>
#include <cmath>
#include <ctime>
#include <vector>
#include <queue>	
#include <utility>	//pair
#include <fstream>	//ofstream
#include <string>
#include <sstream>  //ostringstream
#include "Gsm.h"
#include "Edge.h"
#include "Wcdma.h"
#include "Lte.h"
#include "Bus.h"
#include "distributions.h"
#include "increased_load.h"
#include "sa_scheme.h"

#define ISD_G 1.0									//GSM/EDGE BS InterSideDistance: 1 km				***1.5
#define ISD_W 0.645									//WCDMA BS ISD: 0.645 km							***1
#define ISD_L 0.5									//LTE BS ISD: 0.5 km								***0.75	
#define L_BS1_X (0.5/sqrt(3.0))*cos(120 * PI/180)	//LTE BS#1的x座標
//#define MOVING_RANGE_RADIUS 4*ISD_L				//車子移動範圍圓的半徑
#define NUMBER_OF_UPPER_BOUND_BUS 1					//車子數量
#define NUMBER_OF_GSM_BS 7
#define NUMBER_OF_EDGE_BS 7
#define NUMBER_OF_WCDMA_BS 19
#define NUMBER_OF_LTE_BS 27


using namespace std;

double simulation_time = 20000;											//unit: frame (10 ms)     10min:60000

double GSM_loading_upper_bound = 1.0;
double EDGE_loading_upper_bound = 0.9;
double WCDMA_loading_upper_bound = 0.7;
double LTE_loading_upper_bound = 0.9;

double radius[3] = {ISD_G / (2 * cos(30 * PI/180)), ISD_W / (2 * cos(30 * PI/180)), ISD_L / (2 * cos(30 * PI/180))};

//vector<vector<long long> > bit(NUMBER_OF_BUS, vector<long long>(40,0));
//network(0(network 1),1(network 2),2(network 3),3(network 4)), bs (0,...), traffic type(0,1) 
vector<vector<vector<long long> > > dropped_packet(4, vector<vector<long long>> (NUMBER_OF_LTE_BS, vector<long long> (2, 0)));


double distance_between_points(const pair<double, double> &i, const pair<double, double> &j);
//double dwelling_time(double radius, pair<double, double> bs_position, pair<double, double> bus_position, double bus_speed, vector<pair<double, double>> waypoints, pair<double, double> outer_waypoint);

////ostringstream txt9;
////txt9 << "load_balancing_factor.txt";
////string file_name9 = txt9.str();
//ofstream LOAD_BALANCING_FACTOR("load_balancing_factor.txt");

int main(void)
{
 int seed;
 int total_mt;
 for (seed = 1; seed <= 5; seed++)
 {
  cout << "seed " << seed << endl;
//srand((unsigned)time(NULL));
  srand(seed);
  for (total_mt = 20; total_mt <= 60; total_mt += 10)
   {
	 cout << "total_mt " << total_mt << endl;  
  for(int NUMBER_OF_BUS = 1; NUMBER_OF_BUS <= NUMBER_OF_UPPER_BOUND_BUS; NUMBER_OF_BUS++)
   {
// 	ostringstream txt1;
//  txt1 << "mobility_b1.txt";
//  string file_name1 = txt1.str();
//  ofstream file1(file_name1.c_str());

// 	ostringstream txt2;
// 	txt2 << "mobility_b2.txt";
// 	string file_name2 = txt2.str();
// 	ofstream file2(file_name2.c_str());

// 	ostringstream txt3;
// 	txt3 << "serving_bs_set.txt";
// 	string file_name3 = txt3.str();
// 	ofstream file3(file_name3.c_str());
// 	
// 	ostringstream txt4;
// 	txt4 << "waypoint.txt";
// 	string file_name4 = txt4.str();
// 	ofstream file4(file_name4.c_str());

// 	ostringstream txt5;
// 	txt5 << "WCDMA" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
// 	string file_name5 = txt5.str();
// 	ofstream WCDMA(file_name5.c_str());
// 
// 	ostringstream txt6;
// 	txt6 << "LTE" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
// 	string file_name6 = txt6.str();
// 	ofstream LTE(file_name6.c_str());
// 
// 	ostringstream txt7;
// 	txt7 << "GSM" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
// 	string file_name7 = txt7.str();
// 	ofstream GSM(file_name7.c_str());
// 
// 	ostringstream txt8;
// 	txt8 << "EDGE" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
// 	string file_name8 = txt8.str();
// 	ofstream EDGE(file_name8.c_str());

	ostringstream txt9;
	txt9 << "load_balancing_factor_of_all_BSs" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name9 = txt9.str();
	ofstream LOAD_BALANCING_FACTOR_OF_ALL_BSS(file_name9.c_str());

	ostringstream txt10;
	txt10 << "num_of_handover_in_system" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name10 = txt10.str();
	ofstream NUM_OF_HO_IN_SYSTEM(file_name10.c_str());

	ostringstream txt11;
	txt11 << "avg_voice_packet_delay" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name11 = txt11.str();
	ofstream AVG_VOICE_PKT_DELAY(file_name11.c_str());

	ostringstream txt17;
	txt17 << "avg_video_packet_delay" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name17 = txt17.str();
	ofstream AVG_VIDEO_PKT_DELAY(file_name17.c_str());

	ostringstream txt12;
	txt12 << "avg_voice_drop_rate" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name12 = txt12.str();
	ofstream AVG_VOICE_DROP_RATE(file_name12.c_str());

	ostringstream txt18;
	txt18 << "avg_video_drop_rate" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name18 = txt18.str();
	ofstream AVG_VIDEO_DROP_RATE(file_name18.c_str());

	ostringstream txt13;
	txt13 << "throughput" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name13 = txt13.str();
	ofstream THROUGHPUT(file_name13.c_str());

	ostringstream txt14;
	txt14 << "gho_blocking_ratio" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name14 = txt14.str();
	ofstream GHO_BLOACKING_RATIO(file_name14.c_str());

	ostringstream txt16;
	txt16 << "average_gho_blocking_ratio" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name16 = txt16.str();
	ofstream AVERAGE_GHO_BLOACKING_RATIO(file_name16.c_str());

	ostringstream txt15;
	txt15 << "average_load_balancing_factor_of_all_bss" << "_tot_num_of_mt_" << total_mt << "_seed_" << seed << ".txt";
	string file_name15 = txt15.str();
	ofstream AVERAGE_LOAD_BALANCING_FACTOR_OF_ALL_BSS(file_name15.c_str());

	//0.577, 0.372, 0.288
	
	/*--- 佈建7個gsm的bs ---*/	
	Gsm g_bs[NUMBER_OF_GSM_BS] = 
	{
		Gsm(1,make_pair(0.0,0.0)),							   Gsm(2,make_pair(ISD_G*cos(30*PI/180),ISD_G*sin(30*PI/180))),
		Gsm(3,make_pair(ISD_G*cos(-PI/6),ISD_G*sin(-PI/6))),   Gsm(4,make_pair(0.0,-ISD_G)),
		Gsm(5,make_pair(ISD_G*cos(7*PI/6),ISD_G*sin(7*PI/6))), Gsm(6,make_pair(ISD_G*cos(150*PI/180),ISD_G*sin(150*PI/180))),
		Gsm(7,make_pair(0.0,ISD_G))
	};
	
	/*--- 佈建7個edge的bs ---*/
	Edge e_bs[NUMBER_OF_EDGE_BS] = 
	{
		Edge(1,make_pair(0.0,0.0)),								Edge(2,make_pair(ISD_G*cos(30*PI/180),ISD_G*sin(30*PI/180))),
		Edge(3,make_pair(ISD_G*cos(-PI/6),ISD_G*sin(-PI/6))),	Edge(4,make_pair(0.0,-ISD_G)),
		Edge(5,make_pair(ISD_G*cos(7*PI/6),ISD_G*sin(7*PI/6))),	Edge(6,make_pair(ISD_G*cos(150*PI/180),ISD_G*sin(150*PI/180))),
		Edge(7,make_pair(0.0,ISD_G))
	};

	/*--- 佈建19個wcdma的bs ---*/
	Wcdma w_bs[NUMBER_OF_WCDMA_BS] = 
	{ 
		Wcdma(1,make_pair(0.0,0.0)),								   Wcdma(2,make_pair(ISD_W*cos(30*PI/180),ISD_W*sin(30*PI/180))),
		Wcdma(3,make_pair(ISD_W*cos(-PI/6),ISD_W*sin(-PI/6))),		   Wcdma(4,make_pair(0.0,-ISD_W)), 
		Wcdma(5,make_pair(ISD_W*cos(7*PI/6),ISD_W*sin(7*PI/6))),	   Wcdma(6,make_pair(ISD_W*cos(150*PI/180),ISD_W*sin(150*PI/180))),
		Wcdma(7,make_pair(0.0,ISD_W)),								   Wcdma(8,make_pair(2*ISD_W*cos(30*PI/180),2*ISD_W*sin(30*PI/180))), 
		Wcdma(9,make_pair(2*ISD_W*cos(30*PI/180),0.0)),				   Wcdma(10,make_pair(2*ISD_W*cos(-PI/6),2*ISD_W*sin(-PI/6))),
		Wcdma(11,make_pair(ISD_W*cos(-PI/6),ISD_W*sin(-PI/6)-ISD_W)),  Wcdma(12,make_pair(0.0,-2*ISD_W)),
		Wcdma(13,make_pair(ISD_W*cos(7*PI/6),ISD_W*sin(7*PI/6)-ISD_W)),Wcdma(14,make_pair(2*ISD_W*cos(7*PI/6),2*ISD_W*sin(7*PI/6))),
		Wcdma(15,make_pair(2*ISD_W*cos(150*PI/180),0.0)),			   Wcdma(16,make_pair(2*ISD_W*cos(150*PI/180),2*ISD_W*sin(150*PI/180))),
		Wcdma(17,make_pair(ISD_W*cos(5*PI/6),ISD_W*sin(5*PI/6)+ISD_W)),Wcdma(18,make_pair(0.0,2*ISD_W)),
		Wcdma(19,make_pair(ISD_W*cos(PI/6),ISD_W*sin(PI/6)+ISD_W))
	};

	/*--- 佈建27個lte的bs ---*/
	Lte l_bs[NUMBER_OF_LTE_BS] = 
	{
		Lte(1,make_pair(L_BS1_X,0.25)),													Lte(2,make_pair(L_BS1_X + ISD_L*cos(30*PI/180),0.25 + ISD_L*sin(30*PI/180))),
		Lte(3,make_pair(L_BS1_X + ISD_L*cos(-PI/6),0.25 + ISD_L*sin(-PI/6))),			Lte(4,make_pair(L_BS1_X,0.25 + -ISD_L)), 
		Lte(5,make_pair(L_BS1_X + ISD_L*cos(7*PI/6),0.25 + ISD_L*sin(7*PI/6))),			Lte(6,make_pair(L_BS1_X + ISD_L*cos(150*PI/180),0.25 + ISD_L*sin(150*PI/180))),
		Lte(7,make_pair(L_BS1_X,0.25 + ISD_L)),											Lte(8,make_pair(L_BS1_X + 2*ISD_L*cos(30*PI/180),0.25 + 2*ISD_L*sin(30*PI/180))), 
		Lte(9,make_pair(L_BS1_X + 2*ISD_L*cos(30*PI/180),0.25)),						Lte(10,make_pair(L_BS1_X + 2*ISD_L*cos(-PI/6),0.25 + 2*ISD_L*sin(-PI/6))),
		Lte(11,make_pair(L_BS1_X + ISD_L*cos(-PI/6),0.25 + ISD_L*sin(-PI/6)-ISD_L)),	Lte(12,make_pair(L_BS1_X,0.25 - 2*ISD_L)),
		Lte(13,make_pair(L_BS1_X + ISD_L*cos(7*PI/6),0.25 + ISD_L*sin(7*PI/6)-ISD_L)),	Lte(14,make_pair(L_BS1_X + 2*ISD_L*cos(7*PI/6),0.25 + 2*ISD_L*sin(7*PI/6))),
		Lte(15,make_pair(L_BS1_X + 2*ISD_L*cos(150*PI/180),0.25)),						Lte(16,make_pair(L_BS1_X + 2*ISD_L*cos(150*PI/180),0.25 + 2*ISD_L*sin(150*PI/180))),
		Lte(17,make_pair(L_BS1_X + ISD_L*cos(5*PI/6),0.25 + ISD_L*sin(5*PI/6)+ISD_L)),	Lte(18,make_pair(L_BS1_X,0.25 + 2*ISD_L)),
		Lte(19,make_pair(L_BS1_X + ISD_L*cos(PI/6),0.25 + ISD_L*sin(PI/6)+ISD_L)),		Lte(20,make_pair(L_BS1_X + 3*ISD_L*cos(30*PI/180),0.25 + ISD_L*sin(30*PI/180))),
		Lte(21,make_pair(L_BS1_X + 3*ISD_L*cos(30*PI/180),0.25 + ISD_L*sin(-PI/6))),	Lte(22,make_pair(L_BS1_X + 3*ISD_L*cos(-PI/6),0.25 + 3*ISD_L*sin(-PI/6))),
		Lte(23,make_pair(L_BS1_X + 2*ISD_L*cos(-PI/6),0.25 - 2*ISD_L)),					Lte(24,make_pair(L_BS1_X + ISD_L*cos(-PI/6),0.25 + ISD_L*sin(-PI/6)- 2*ISD_L)),
		Lte(25,make_pair(L_BS1_X,0.25 - 3*ISD_L)),										Lte(26,make_pair(L_BS1_X + ISD_L*cos(7*PI/6),0.25 + ISD_L*sin(7*PI/6)- 2*ISD_L)), 
		Lte(27,make_pair(L_BS1_X + 2*ISD_L*cos(150*PI/180),0.25 - 2*ISD_L))
	};

	/*--- 將產生的gsm, wcdma, lte所有的bs座標複製收集起來(傳入之後產生的bus,bus會再告訴它產生的mt) ---*/	
	vector<vector<pair<double, double> > > bs_location_container(3);  
//	bs_location_container.assign(3, vector<pair<double,double> >(0, pair<double, double>(0, 0) ));
//	vector<pair<double, double> > g_bs_location_container, w_bs_location_container, l_bs_location_container;
	
	for(int k = 0; k < 7; k++)
	{ bs_location_container[0].push_back(g_bs[k].get_bs_location()); }	
	for(int k = 0; k < 19; k++)
	{ bs_location_container[1].push_back(w_bs[k].get_bs_location()); }	
	for(int k = 0; k < 27; k++)
	{ bs_location_container[2].push_back(l_bs[k].get_bs_location()); }	

// 	bs_location_container.push_back(g_bs_location_container); //如果bs_location_container後面已經有(3)再寫push_back會push第四個出來
// 	bs_location_container.push_back(w_bs_location_container);
// 	bs_location_container.push_back(l_bs_location_container);	

	/*--- 產生車子,車子物件內會產生mts ---*/
	vector<Bus> bus;
		
	for(int b = 0; b < NUMBER_OF_BUS; b++)
	{
		bus.push_back(Bus(b + 1, total_mt, 36, bs_location_container));	//40:40個MT, 36:36 km/hr 
	}

/************************ test the difference of life time *********************/
// 	for(int b = 0; b < NUMBER_OF_BUS; b++)
// 	{
// 		for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)
// 		{
// 			cout << bus[b].alias_of_mt_container()[i].get_life_time() << endl;
// 		}
// 		cout << endl;
// 	}
// 	system("PAUSE");
/*****************************************************************************/

	/*--- 決定車內mt的network,BS並更改該BS的loading ---*/										
	for(int b = 0; b < NUMBER_OF_BUS; b++)							//每台車子
	{
		/*--- 找出每個網路中訊號最強的bs ---*/
		int g_k = -1, w_k = -1, l_k = -1;
		vector<double> g_bs_rx_pilot_power, w_bs_rx_pilot_power, l_bs_rx_pilot_power;
		g_bs_rx_pilot_power.push_back(-100), w_bs_rx_pilot_power.push_back(-100), l_bs_rx_pilot_power.push_back(-100);

		//gsm
 		for(unsigned int index = 0; index < bs_location_container[0].size(); index++)							
		{
			double d = distance_between_points(bus[b].get_bus_location(), bs_location_container[0][index]);
			double g_tx_pilot_power = 31;	
			g_bs_rx_pilot_power.push_back(g_tx_pilot_power - gsm_edge_path_loss(d));
			if(g_bs_rx_pilot_power[index + 1] > g_bs_rx_pilot_power[0])
			{
				g_bs_rx_pilot_power[0] = g_bs_rx_pilot_power[index + 1];				
				g_k = index + 1;
			}
		}
		/*cout << g_bs_rx_pilot_power[0] << " " ;*/

		/*--- 更新bus的serving bs set ---*/
		bus[b].get_serving_bs_set()[0] = g_k;
		
		//wcdma
		for(unsigned int index = 0; index < bs_location_container[1].size(); index++)							
		{
			double d = distance_between_points(bus[b].get_bus_location(), bs_location_container[1][index]);
			double w_tx_pilot_power = 33;	
			w_bs_rx_pilot_power.push_back(w_tx_pilot_power - wcdma_path_loss(d));
			if(w_bs_rx_pilot_power[index + 1] > w_bs_rx_pilot_power[0])
			{
				w_bs_rx_pilot_power[0] = w_bs_rx_pilot_power[index + 1];
				w_k = index + 1;
			}
		}
		/*	cout << w_k << " ";*/

		/*--- 更新bus的serving bs set ---*/
		bus[b].get_serving_bs_set()[1] = w_k;
		
		//lte
		for(unsigned int index = 0; index < bs_location_container[2].size(); index++)							
		{
			double d = distance_between_points(bus[b].get_bus_location(), bs_location_container[2][index]);
			double l_tx_pilot_power = 46;	
			l_bs_rx_pilot_power.push_back(l_tx_pilot_power - lte_path_loss(d));
			if(l_bs_rx_pilot_power[index + 1] > l_bs_rx_pilot_power[0])
			{
				l_bs_rx_pilot_power[0] = l_bs_rx_pilot_power[index + 1];
				l_k = index + 1;
			}
		}
		/*cout << l_bs_rx_pilot_power[0] << " " ;*/

		/*--- 更新bus的serving bs set ---*/
		bus[b].get_serving_bs_set()[2] = l_k;
		
		/*--- 隨機選加入後不超過upper bound的網路 ---*/
		for(int i = 0; i < bus[b].get_num_of_mt(); i++)									//每個mt
		{
			cout << "mt " << bus[b].alias_of_mt_container()[i].get_mt_number() << " " << "h " << bus[b].alias_of_mt_container()[i].get_traffic_type() << " " ;

			int j = -1, k = -1;
			vector<int> network_number;
			int type = bus[b].alias_of_mt_container()[i].get_traffic_type();
			if(type == 0)				//voice
			{  	
				for(int temp_j = 0; temp_j < 3; temp_j++)
				{
					if(g_bs[g_k - 1].get_load_intensity() + g_increased_load(0) <= GSM_loading_upper_bound && temp_j == 0)
					{
						network_number.push_back(1);
					}
					else if(w_bs[w_k - 1].get_load_intensity() + w_increased_load(0) <= WCDMA_loading_upper_bound && temp_j == 1)
					{
						network_number.push_back(3);
					}
					else if(l_bs[l_k - 1].get_load_intensity() + l_increased_load(0) <= LTE_loading_upper_bound && temp_j == 2)
					{
						network_number.push_back(4);
					}
				}
				
				if(network_number.size() != 0)
				{
					j = network_number[unidrnd(0, network_number.size() - 1)];
				}
				else
				{
					j = 0;
				}

				if(j == 0)
				{
					k = 0;
					cout << " haven't connect network " << endl;
				}
				else if(j == 1)	
				{
					k = g_k;															//選j = 1裡訊號最強的bs
					g_bs[k-1].get_load_intensity() += g_increased_load(0);				//修改網路內部的load intensity
					cout << "select g_bs[" << k-1 << "] " << g_bs[k-1].get_load_intensity() << endl;
				}
				else if(j == 3)	
				{
					k = w_k;															//選j = 3裡訊號最強的bs
					w_bs[k-1].get_load_intensity() += w_increased_load(0);				//修改網路內部的load intensity
					cout << "select w_bs[" << k-1 << "] " << w_bs[k-1].get_load_intensity() << endl;
				}
				else if(j == 4)
				{
					k = l_k;															//選j = 4裡訊號最強的bs
					l_bs[k-1].get_load_intensity() += l_increased_load(0);				//修改網路內部的load intensity
					cout << "select l_bs[" << k-1 << "] " << l_bs[k-1].get_load_intensity() << endl;
				}
			}

			else	//video, http ,ftp
			{
				/*--- 檢查serving bs set可否進入 ---*/
				for(int temp_j = 0; temp_j < 3; temp_j++)
				{
					if(e_bs[g_k - 1].get_load_intensity() + e_increased_load(type) <= EDGE_loading_upper_bound && temp_j == 0)
					{
						network_number.push_back(2);
					}
					else if(w_bs[w_k - 1].get_load_intensity() + w_increased_load(type) <= WCDMA_loading_upper_bound && temp_j == 1)
					{
						network_number.push_back(3);
					}
					else if(l_bs[l_k - 1].get_load_intensity() + l_increased_load(type) <= LTE_loading_upper_bound && temp_j == 2)
					{
						network_number.push_back(4);
					}
				}
				/*--- 若有serving bs set可進入 ---*/
				if(network_number.size() != 0)
				{
					/*--- 隨機選一個 ---*/
					j = network_number[unidrnd(0, network_number.size() - 1)];
				}
				/*--- 若沒有serving bs set可進入 ---*/
				else
				{
					/*--- block ---*/
					j = 0;
				}
				/*---若 j = 0, 2, 3, or 4 ---*/
				if(j == 0)
				{
					k = 0;
					cout << " haven't connect network " << endl;
				}
				else if(j == 2)
				{
					k = g_k;	//選j = 2裡訊號最強的bs	
					e_bs[k-1].get_load_intensity() += e_increased_load(type);
					cout << "select e_bs[" << k-1 << "] " << e_bs[k-1].get_load_intensity() << endl;
				}
				else if(j == 3)
				{
					k = w_k;	//選j = 3裡訊號最強的bs
					w_bs[k-1].get_load_intensity() += w_increased_load(type);
					cout << "select w_bs[" << k-1 << "] " << w_bs[k-1].get_load_intensity() << endl;
				}
				else if(j == 4)
				{
					k = l_k;	//選j = 4裡訊號最強的bs
					l_bs[k-1].get_load_intensity() += l_increased_load(type);
					cout << "select l_bs[" << k-1 << "] " << l_bs[k-1].get_load_intensity() << endl;
				}
			}

			/*--- 修改車子裡的mt的serving網路及bs的編號 ---*/

			bus[b].alias_of_mt_container()[i].get_network_bs_number().first = j;
			bus[b].alias_of_mt_container()[i].get_network_bs_number().second = k;		
		}		
	}

/************************************ serving network, bs ************************************************/
// 	for(int b = 0; b < NUMBER_OF_BUS; b++)					//每台車子
// 	{
// 		cout << "num_of_mt_in_bus " << b+1 << ": " << bus[b].alias_of_mt_container().size() << endl;
// 		
// 		
// 		for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)
// 			cout << " No." << bus[b].alias_of_mt_container()[i].get_mt_number() << " "
// 				 << " type " << bus[b].alias_of_mt_container()[i].get_traffic_type() << " "
// 				 << " serving (net, bs) "
// 				 <<	"("<< bus[b].alias_of_mt_container()[i].get_network_bs_number().first << ", " 
// 				       << bus[b].alias_of_mt_container()[i].get_network_bs_number().second << ")" << endl;
// 	}
// 	system("PAUSE");

/**************************************** load intensity ************************************************/
// 	for(int k = 0; k < NUMBER_OF_GSM_BS; k++)
// 		cout << "bs " << g_bs[k].get_bs_number()<< " intensity: " << g_bs[k].get_load_intensity() << endl;
// 	cout << endl;
// 	for(int k = 0; k < NUMBER_OF_EDGE_BS; k++)
// 		cout << "bs " << e_bs[k].get_bs_number()<< " intensity: " << e_bs[k].get_load_intensity() << endl;
// 	cout << endl;
// 	for(int k = 0; k < NUMBER_OF_WCDMA_BS; k++)
// 		cout << "bs " << w_bs[k].get_bs_number()<< " intensity: " << w_bs[k].get_load_intensity() << endl;
// 	cout << endl;
// 	for(int k = 0; k < NUMBER_OF_LTE_BS; k++)
// 		cout << "bs " << l_bs[k].get_bs_number()<< " intensity: " << l_bs[k].get_load_intensity() << endl;
// 	system("PAUSE");
/*********************************************************************************************************/

/***************************************** increased load ************************************************/
// 	cout << "increased_load " << endl;
// 	for(int h = 0; h < 4; h++)
// 	{
// 		for(int j = 1; j <=4 ; j++)
// 			cout << "h " << h << " j " << j << " " << increased_load(h, j) << endl; 
// 	}
// 	cout << "g increased load " << endl;
// 	for(int h = 0; h < 1; h++)	
// 		cout << g_increased_load(h) << endl;
// 	cout << "e increased load " << endl;
// 	for(int h = 1; h < 4; h++)	
// 		cout << e_increased_load(h) << endl;
// 	cout << "w increased load " << endl;
// 	for(int h = 0; h < 4; h++)	
// 		cout << w_increased_load(h) << endl;
// 	cout << "l increased load " << endl;
// 	for(int h = 0; h < 4; h++)	
// 		cout << l_increased_load(h) << endl;
// 	system("PAUSE");

/*********************************************************************************************************/
	
	/*---bs#, bus#, mt#---*/
	vector<vector<vector<int> > > g_list, e_list, w_list, l_list;				//resource allocation list for the MTs in the bus in each BS

	/*--- bs#, mt# ---*/
	vector<vector<int> > g_fixed_list, e_fixed_list, w_fixed_list, l_fixed_list;//resource allocation list for fixed MTs in each BS

	/*--- bus#(1,2,...), traffic type(0,1,2,3), MT index(0,1,...,39)) ---*/
	vector<vector<vector<int> > > gho_list;
	
	int f;  
	int gsm_frame = 0;

	int num_of_handover_in_system = 0;
	double sum_of_voice_packet_delay = 0, sum_of_video_packet_delay = 0;
	long long sum_of_voice_suc_packet = 0, sum_of_video_suc_packet = 0;
	long long sum_of_voice_drop_packet = 0, sum_of_video_drop_packet = 0;
	long long sum_of_trans_bit = 0;
	double sum_of_average_load_intensity = 0;
	double sum_of_gho_blocking_ratio = 0;
	double sum_of_load_balancing_factor = 0;
	int cumulated_times_of_gho = 0;

	vector<vector<int> > counter(NUMBER_OF_BUS, vector<int> (total_mt, 0));   //被block的mt停100個frame在重新連網路
	
	/*--- 改變時間 ---*/
	for(f = 0; f <= simulation_time; f++)																		//第幾個frame
	{	
		g_list.assign(NUMBER_OF_GSM_BS, vector<vector<int> > (NUMBER_OF_BUS, vector<int> (0)));
		e_list.assign(NUMBER_OF_EDGE_BS, vector<vector<int> > (NUMBER_OF_BUS, vector<int> (0)));
		w_list.assign(NUMBER_OF_WCDMA_BS, vector<vector<int> > (NUMBER_OF_BUS, vector<int> (0)));
		l_list.assign(NUMBER_OF_LTE_BS, vector<vector<int> > (NUMBER_OF_BUS, vector<int> (0)));
		
		g_fixed_list.assign(NUMBER_OF_GSM_BS, vector<int> (0));
		e_fixed_list.assign(NUMBER_OF_EDGE_BS, vector<int> (0));
		w_fixed_list.assign(NUMBER_OF_WCDMA_BS, vector<int> (0));
		l_fixed_list.assign(NUMBER_OF_LTE_BS, vector<int> (0));

		/*--- 若bs中有fixed mt網路為0, 自己重新連上原網路 ---*/

		/*--- 若bus上有mt的網路為0,自己重新選擇新網路 ---*/
		for(int b = 0; b < NUMBER_OF_BUS; b++)
		{
			for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)
			{
				if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 0 && counter[b][i] < 500)
				{
					counter[b][i] ++;
				}
				else if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 0 && counter[b][i] == 500)
				{
					/*選網路*/
					int j = -1, k = -1;
					vector<int> network_number;
					int type = bus[b].alias_of_mt_container()[i].get_traffic_type();
					if(type == 0)				//voice
					{  	
						for(int temp_j = 0; temp_j < 3; temp_j++)
						{
							if(g_bs[bus[b].get_serving_bs_set()[0] - 1].get_load_intensity() + g_increased_load(0) <= GSM_loading_upper_bound && temp_j == 0)
							{
								network_number.push_back(1);
							}
							else if(w_bs[bus[b].get_serving_bs_set()[1] - 1].get_load_intensity() + w_increased_load(0) <= WCDMA_loading_upper_bound && temp_j == 1)
							{
								network_number.push_back(3);
							}
							else if(l_bs[bus[b].get_serving_bs_set()[2] - 1].get_load_intensity() + l_increased_load(0) <= LTE_loading_upper_bound && temp_j == 2)
							{
								network_number.push_back(4);
							}
						}

						if(network_number.size() != 0)
						{
							j = network_number[unidrnd(0, network_number.size() - 1)];
						}
						else
						{
							j = 0;
						}

						if(j == 0)
						{
							k = 0;
						}
						else if(j == 1)
						{
							k = bus[b].get_serving_bs_set()[0];									//選j = 1裡訊號最強的bs
							g_bs[k-1].get_load_intensity() += g_increased_load(0);				//修改網路內部的load intensity
						}
						else if(j == 3)
						{
							k = bus[b].get_serving_bs_set()[1];									//選j = 3裡訊號最強的bs
							w_bs[k-1].get_load_intensity() += w_increased_load(0);				//修改網路內部的load intensity
						}
						else if(j == 4)
						{
							k = bus[b].get_serving_bs_set()[2];									//選j = 4裡訊號最強的bs
							l_bs[k-1].get_load_intensity() += l_increased_load(0);				//修改網路內部的load intensity
						}
					}
					else	//video, http ,ftp
					{
						for(int temp_j = 0; temp_j < 3; temp_j++)
						{
							if(e_bs[bus[b].get_serving_bs_set()[0] - 1].get_load_intensity() + e_increased_load(type) <= EDGE_loading_upper_bound && temp_j == 0)
							{
								network_number.push_back(2);
							}
							else if(w_bs[bus[b].get_serving_bs_set()[1] - 1].get_load_intensity() + w_increased_load(type) <= WCDMA_loading_upper_bound && temp_j == 1)
							{
								network_number.push_back(3);
							}
							else if(l_bs[bus[b].get_serving_bs_set()[2] - 1].get_load_intensity() + l_increased_load(type) <= LTE_loading_upper_bound && temp_j == 2)
							{
								network_number.push_back(4);
							}
						}

						if(network_number.size() != 0)
						{
							j = network_number[unidrnd(0, network_number.size() - 1)];
						}
						else
						{
							j = 0;
						}

						if(j == 0)
						{
							k = 0;
						}
						else if(j == 2)
						{
							k = bus[b].get_serving_bs_set()[0];	//選j = 2裡訊號最強的bs	
							e_bs[k-1].get_load_intensity() += e_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type());
						}
						else if(j ==3)
						{
							k = bus[b].get_serving_bs_set()[1];	//選j = 3裡訊號最強的bs
							w_bs[k-1].get_load_intensity() += w_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type());
						}
						else if(j ==4)
						{
							k = bus[b].get_serving_bs_set()[2];	//選j = 4裡訊號最強的bs
							l_bs[k-1].get_load_intensity() += l_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type());
						}
					}

					/*--- 修改車子裡的mt的serving網路, bs的編號, life time ---*/

					bus[b].alias_of_mt_container()[i].get_network_bs_number().first = j;
					bus[b].alias_of_mt_container()[i].get_network_bs_number().second = k;

					bus[b].alias_of_mt_container()[i].get_life_time() = exprnd(15000 * 10e-3);
					
					/*reset*/
					counter[b][i] = 0; 
				}
			}
		}

		/*--- 每個BS的fixed MTs產生資料 ---*/
		for(int B = 0; B < NUMBER_OF_GSM_BS; B++)	//gsm
		{
			for(unsigned int i = 0; i < g_bs[B].get_gsm_fixed_mt().size(); i++)
			{
				g_bs[B].get_gsm_fixed_mt()[i].data_generator(f);
				g_bs[B].get_gsm_fixed_mt()[i].delay_constraint_check(f, dropped_packet);
			}
		}
		for(int B = 0; B < NUMBER_OF_EDGE_BS; B++)	//EDGE
		{
			for(unsigned int i = 0; i < e_bs[B].get_edge_fixed_mt().size(); i++)
			{
				e_bs[B].get_edge_fixed_mt()[i].data_generator(f);
				e_bs[B].get_edge_fixed_mt()[i].delay_constraint_check(f, dropped_packet);	
			}
		}
		for(int B = 0; B < NUMBER_OF_WCDMA_BS; B++)	//WCDMA
		{
			for(unsigned int i = 0; i < w_bs[B].get_wcdma_fixed_mt().size(); i++)
			{
				w_bs[B].get_wcdma_fixed_mt()[i].data_generator(f);
				w_bs[B].get_wcdma_fixed_mt()[i].delay_constraint_check(f, dropped_packet);
				if(w_bs[B].get_wcdma_fixed_mt()[i].need_to_transmit(f) == true)
				{
					w_fixed_list[B].push_back(w_bs[B].get_wcdma_fixed_mt()[i].get_mt_number());
				}
			}
		}
		for(int B = 0; B < NUMBER_OF_LTE_BS; B++)	//LTE
		{
			for(unsigned int i = 0; i < l_bs[B].get_lte_fixed_mt().size(); i++)
			{
				l_bs[B].get_lte_fixed_mt()[i].data_generator(f);
				l_bs[B].get_lte_fixed_mt()[i].delay_constraint_check(f, dropped_packet);
				if(l_bs[B].get_lte_fixed_mt()[i].need_to_transmit(f) == true)
				{
					l_fixed_list[B].push_back(l_bs[B].get_lte_fixed_mt()[i].get_mt_number());
					l_bs[B].get_lte_fixed_mt()[i].update_busy_time();
				}
			}
		}

		/*--- 每台Bus的MTs產生資料 ---*/
		for(int b = 0; b < NUMBER_OF_BUS; b++)														//每台bus
		{
			for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)					//bus上的每個MT ; i < bus[b].alias_of_mt_container().size()
			{
				if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first != 0)			//若沒被block
				{
					bus[b].alias_of_mt_container()[i].data_generator(f);							//產生資料
					bus[b].alias_of_mt_container()[i].delay_constraint_check(f, dropped_packet);	//把delay超過限制的封包丟掉

					if(bus[b].alias_of_mt_container()[i].need_to_transmit(f) == true)
					{
						/*--- 把要傳的mt依照所屬的網路,bs分類 ---*/
						if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 1)
						{
//							g_list[bus[b].alias_of_mt_container()[i].get_network_bs_number().second - 1][b].push_back(bus[b].alias_of_mt_container()[i].get_mt_number()); 
							bus[b].alias_of_mt_container()[i].update_busy_time();
//							bus[b].alias_of_mt_container()[i].update_edge_busy_frame();
						}
						else if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 2)
						{
//							e_list[bus[b].alias_of_mt_container()[i].get_network_bs_number().second - 1][b].push_back(bus[b].alias_of_mt_container()[i].get_mt_number()); 
							bus[b].alias_of_mt_container()[i].update_busy_time();
//							bus[b].alias_of_mt_container()[i].update_edge_busy_frame();
						}
						if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 3)
						{
							w_list[bus[b].alias_of_mt_container()[i].get_network_bs_number().second - 1][b].push_back(bus[b].alias_of_mt_container()[i].get_mt_number()); 
							bus[b].alias_of_mt_container()[i].update_busy_time();	
//							bus[b].alias_of_mt_container()[i].update_edge_busy_frame();
						}
						else if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 4)
						{
							l_list[bus[b].alias_of_mt_container()[i].get_network_bs_number().second - 1][b].push_back(bus[b].alias_of_mt_container()[i].get_mt_number()); 
							bus[b].alias_of_mt_container()[i].update_busy_time();
//							bus[b].alias_of_mt_container()[i].update_edge_busy_frame();
						}
					}
				}
				
			}
		}

/****************************************************** buffer data *****************************************************************/
// 		for(int b = 0; b < NUMBER_OF_BUS; b++)													
// 		{
// 			cout << "bus " << b+1 << endl; 
// 			for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)				
// 			{
// 				cout << " i: " << bus[b].alias_of_mt_container()[i].get_mt_number()
// 				     << " h: " << bus[b].alias_of_mt_container()[i].get_traffic_type() 
// 					 << " Buffer size: " << bus[b].alias_of_mt_container()[i].get_mt_buffer().size()
// 				     << " front.size: " << bus[b].alias_of_mt_container()[i].get_mt_buffer().front().get_size() 
// 					 << " front.atime: " << bus[b].alias_of_mt_container()[i].get_mt_buffer().front().get_arrival_time() << endl;
// 			}
// 			cout << endl;
// 		}
/************************************************************************************************************************************/


/******************************************** transmitting list *********************************************************/
// 		for(int B = 0; B < NUMBER_OF_GSM_BS; B++)
// 		{
// 			cout << "gsm bs " << B + 1 << endl;
// 			for(int b = 0; b < NUMBER_OF_BUS; b++)													
// 			{
// 				cout << "bus " << b + 1 << endl; 
// 				for(unsigned int i = 0; i < g_list[B][b].size(); i++)			
// 				{
// 					cout << g_list[B][b][i] << " ";
// 				}
// 				cout << endl;
// 			}
// 		}
// 		for(int B = 0; B < NUMBER_OF_EDGE_BS; B++)
// 		{
// 			cout << "edgs bs " << B + 1 << endl;
// 			for(int b = 0; b < NUMBER_OF_BUS; b++)													
// 			{
// 				cout << "bus " << b + 1 << endl; 
// 				for(unsigned int i = 0; i < e_list[B][b].size(); i++)			
// 				{
// 					cout << e_list[B][b][i] << " ";
// 				}
// 				cout << endl;
// 			}
// 		}
// 		for(int B = 0; B < NUMBER_OF_WCDMA_BS; B++)
// 		{
// 			cout << "wcdma bs " << B + 1 << endl;
// 			for(int b = 0; b < NUMBER_OF_BUS; b++)													
// 			{
// 				cout << "bus " << b + 1 << endl; 
// 				for(unsigned int i = 0; i < w_list[B][b].size(); i++)			
// 				{
// 					cout << w_list[B][b][i] << " ";
// 				}
// 				cout << endl;
// 			}
// 		}
// 		for(int B = 0; B < NUMBER_OF_LTE_BS; B++)
// 		{
// 			cout << "lte bs " << B + 1 << endl;
// 			for(int b = 0; b < NUMBER_OF_BUS; b++)													
// 			{
// 				cout << "bus " << b + 1 << endl; 
// 				for(unsigned int i = 0; i < l_list[B][b].size(); i++)			
// 				{
// 					cout << l_list[B][b][i] << " ";
// 				}
// 				cout << endl;
// 			}
// 		}
// 		system("PAUSE");
/**************************************************************************************************************/
		
		/*--- bs消耗Bus裡被此bs服務的MTs的資料 ---*/
		while((gsm_frame + 1) * 4.615e-3 < (f + 1) * 10e-3)											
		{
			for(int b = 0; b < NUMBER_OF_BUS; b++)													//每台bus
			{
				for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)				//bus上的每個MT ; i < bus[b].alias_of_mt_container().size()
				{
					if(bus[b].alias_of_mt_container()[i].edge_need_to_transmit(gsm_frame) == true && 
						bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 1)
					{
						g_list[bus[b].alias_of_mt_container()[i].get_network_bs_number().second - 1][b].push_back(bus[b].alias_of_mt_container()[i].get_mt_number()); 
						bus[b].alias_of_mt_container()[i].update_edge_busy_frame();
					}
					if(bus[b].alias_of_mt_container()[i].edge_need_to_transmit(gsm_frame) == true && 
					   bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 2)
					{
						e_list[bus[b].alias_of_mt_container()[i].get_network_bs_number().second - 1][b].push_back(bus[b].alias_of_mt_container()[i].get_mt_number()); 
						bus[b].alias_of_mt_container()[i].update_edge_busy_frame();
					}
					if(bus[b].alias_of_mt_container()[i].edge_need_to_transmit(gsm_frame) == true && 
						bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 3)
					{
						bus[b].alias_of_mt_container()[i].update_edge_busy_frame();
					}
					if(bus[b].alias_of_mt_container()[i].edge_need_to_transmit(gsm_frame) == true && 
						bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 4)
					{
						bus[b].alias_of_mt_container()[i].update_edge_busy_frame();
					}
				}
			}

			for(int B = 0; B < NUMBER_OF_GSM_BS; B++)
			{
				for(unsigned int i = 0; i < g_bs[B].get_gsm_fixed_mt().size(); i++)
				{
					if(g_bs[B].get_gsm_fixed_mt()[i].need_to_transmit(f) == true)
					{
						g_fixed_list[B].push_back(g_bs[B].get_gsm_fixed_mt()[i].get_mt_number());
						g_bs[B].get_gsm_fixed_mt()[i].update_edge_busy_frame();
					}
				}
				
				g_bs[B].resource_allocation(gsm_frame, g_list[B], bus, g_fixed_list[B]);
			}

			for(int B = 0; B < NUMBER_OF_EDGE_BS; B++)
			{
				for(unsigned int i = 0; i < e_bs[B].get_edge_fixed_mt().size(); i++)
				{
					if(e_bs[B].get_edge_fixed_mt()[i].need_to_transmit(f) == true)
					{
						e_fixed_list[B].push_back(e_bs[B].get_edge_fixed_mt()[i].get_mt_number());
						e_bs[B].get_edge_fixed_mt()[i].update_edge_busy_frame();
					}
				}

				e_bs[B].resource_allocation(gsm_frame, e_list[B], bus, e_fixed_list[B]);
			}
						
			/************************************************ data saving **************************************************************/
//*** 			GSM << "gsm_frame " << gsm_frame << endl;
// 			EDGE << "edge_frame " << gsm_frame << endl;
			
// 			for(int B = 0; B < NUMBER_OF_GSM_BS; B++)
// 			{
// 				GSM << " BS# " << B + 1 
// 					<< " tx bit " << g_bs[B].get_gsm_transmitted_bit()
// 					<< " suc pkt " << g_bs[B].get_gsm_successful_packet()
// 					<< " drop pkt " << dropped_packet[0][B][0]
// 				    << " delay " << g_bs[B].get_gsm_packet_delay()
// 					<< " avg delay " << g_bs[B].get_gsm_packet_delay() / g_bs[B].get_gsm_successful_packet() << endl;
// 				EDGE << " BS# " << B + 1 
// 					 << " tx bit " << e_bs[B].get_edge_transmitted_bit()[0] + e_bs[B].get_edge_transmitted_bit()[1] + e_bs[B].get_edge_transmitted_bit()[2] 
// 				     << " suc pkt " << e_bs[B].get_edge_successful_packet()[0] + e_bs[B].get_edge_successful_packet()[1] + e_bs[B].get_edge_successful_packet()[2]
// 					 << " drop pkt " << dropped_packet[1][B][1] 
// 					 << " delay " << e_bs[B].get_edge_packet_delay()[0]
// 					 << " avg delay " << e_bs[B].get_edge_packet_delay()[0] / e_bs[B].get_edge_successful_packet()[0] << endl;
//*** 			}
			
// 			for(int b = 0; b < NUMBER_OF_BUS; b++)
// 			{
// 				for(int i = 0; i < bus[b].get_num_of_mt(); i++)
// 				{
// 					if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 1)
// 					{
// 						GSM << "i " << i << "  " << "h " << bus[b].alias_of_mt_container()[i].get_traffic_type() 
// 							<< "  " << "tx bit " << bus[b].alias_of_mt_container()[i].get_transmitted_bits() 
// 							<< "  " << "suc pkt " << bus[b].alias_of_mt_container()[i].get_successful_packet()
// 							<< "  " << "drop pkt " << bus[b].alias_of_mt_container()[i].get_dropped_packet() 
// 							<< "  " << "delay " << bus[b].alias_of_mt_container()[i].get_packet_delay() 	  
// 							<< "  " << "avg delay " << bus[b].alias_of_mt_container()[i].get_packet_delay() / bus[b].alias_of_mt_container()[i].get_successful_packet() << endl;
// 					}

// 					if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 2)
// 					{
// 						if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 0 || bus[b].alias_of_mt_container()[i].get_traffic_type() == 1)
// 						{
// 							EDGE << "i " << i << "  " << "h " << bus[b].alias_of_mt_container()[i].get_traffic_type() 
// 								 << "  " << "tx bit " << bus[b].alias_of_mt_container()[i].get_transmitted_bits() 
// 								 << "  " << "suc pkt " << bus[b].alias_of_mt_container()[i].get_successful_packet()
// 								 << "  " << "drop pkt " << bus[b].alias_of_mt_container()[i].get_dropped_packet() 
// 								 << "  " << "delay " << bus[b].alias_of_mt_container()[i].get_packet_delay() 	  
// 								 << "  " << "avg delay " << bus[b].alias_of_mt_container()[i].get_packet_delay() / bus[b].alias_of_mt_container()[i].get_successful_packet() << endl;
// 						}
// 						else
// 						{
// 							EDGE << "i " << i << "  " << "h " << bus[b].alias_of_mt_container()[i].get_traffic_type() 
// 								 << "  " << "tx bit " << bus[b].alias_of_mt_container()[i].get_transmitted_bits() 
// 								 << "  " << "suc pkt " << bus[b].alias_of_mt_container()[i].get_successful_packet()
// 								 << "  " << "drop pkt " << bus[b].alias_of_mt_container()[i].get_dropped_packet() << endl;
// 						}
// 					}
// 				}
//			}
			/***************************************************************************************************************************/
			gsm_frame++;
		}

		for(int B = 0; B < NUMBER_OF_WCDMA_BS; B++)
		{
			if (w_list[B].size() == 0 && w_fixed_list[B].size() == 0)							//no users in cell B
				continue;
			w_bs[B].resource_allocation(f, w_list[B], bus, w_fixed_list[B]);		//舊寫法(錯):w_bs[B].resource_allocation(f, transmittable_list, bus);
		}
		for(int B = 0; B < NUMBER_OF_LTE_BS; B++)
		{
			if (l_list[B].size() == 0 && l_fixed_list[B].size() == 0)							//no users in cell B
				continue;
			l_bs[B].resource_allocation(f, l_list[B], bus, l_fixed_list[B]);
		}

/********************************************************* data saving ************************************************************/	
//*** 		WCDMA << "wcdma_frame " << f << endl;
// 		LTE << "lte_frame " << f << endl;
// 		for(int B = 0; B < NUMBER_OF_WCDMA_BS; B++)
// 		{
// 			WCDMA << " BS# " << B + 1 
// 				  << " tx bit " << w_bs[B].get_wcdma_transmitted_bit()[0] + w_bs[B].get_wcdma_transmitted_bit()[1] + w_bs[B].get_wcdma_transmitted_bit()[2] + w_bs[B].get_wcdma_transmitted_bit()[3] 
// 				  << " suc pkt " << w_bs[B].get_wcdma_successful_packet()[0] + w_bs[B].get_wcdma_successful_packet()[1] + w_bs[B].get_wcdma_successful_packet()[2] +w_bs[B].get_wcdma_successful_packet()[3]
// 				  << " drop pkt " << dropped_packet[2][B][0] + dropped_packet[2][B][1]
// 				  << " delay " << w_bs[B].get_wcdma_packet_delay()[0] + w_bs[B].get_wcdma_packet_delay()[1] 
// 				  << " avg delay " << (w_bs[B].get_wcdma_packet_delay()[0] + w_bs[B].get_wcdma_packet_delay()[1]) / (w_bs[B].get_wcdma_successful_packet()[0] + w_bs[B].get_wcdma_successful_packet()[1]) << endl; 
// 		}
// 		
// 		for(int B = 0 ; B < NUMBER_OF_LTE_BS; B++)
// 		{
// 			LTE << " BS# " << B + 1
// 				<< " tx bit " << l_bs[B].get_lte_transmitted_bit()[0] + l_bs[B].get_lte_transmitted_bit()[1] + l_bs[B].get_lte_transmitted_bit()[2] + l_bs[B].get_lte_transmitted_bit()[3]
// 			    << " suc pkt " << l_bs[B].get_lte_successful_packet()[0] + l_bs[B].get_lte_successful_packet()[1] + l_bs[B].get_lte_successful_packet()[2] +l_bs[B].get_lte_successful_packet()[3]
// 				<< " drop pkt " << dropped_packet[3][B][0] + dropped_packet[3][B][1]
// 				<< " delay " << l_bs[B].get_lte_packet_delay()[0] + l_bs[B].get_lte_packet_delay()[1]
// 				<< " avg delay " << (l_bs[B].get_lte_packet_delay()[0] + l_bs[B].get_lte_packet_delay()[1]) / (l_bs[B].get_lte_successful_packet()[0] + l_bs[B].get_lte_successful_packet()[1]) << endl;
//*** 		}

//		for(int b = 0; b < NUMBER_OF_BUS; b++)
//		{
//			for(int i = 0; i < bus[b].get_num_of_mt(); i++)
//			{	
// 				if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 3)
// 				{
// 					if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 0 || bus[b].alias_of_mt_container()[i].get_traffic_type() == 1)
// 					{
// 						WCDMA << "i " << i << "  " << "h " << bus[b].alias_of_mt_container()[i].get_traffic_type() 
// 							  << "  " << "tx bit " << bus[b].alias_of_mt_container()[i].get_transmitted_bits() 
// 							  << "  " << "suc pkt " << bus[b].alias_of_mt_container()[i].get_successful_packet()
// 							  << "  " << "drop pkt " << bus[b].alias_of_mt_container()[i].get_dropped_packet() 
// 							  << "  " << "delay " << bus[b].alias_of_mt_container()[i].get_packet_delay() 	  
// 							  << "  " << "avg delay " << bus[b].alias_of_mt_container()[i].get_packet_delay() / bus[b].alias_of_mt_container()[i].get_successful_packet() << endl;			  
// 					}
// 					else
// 					{
// 						WCDMA << "i " << i << "  " << "h " << bus[b].alias_of_mt_container()[i].get_traffic_type() 
// 							  << "  " << "tx bit " << bus[b].alias_of_mt_container()[i].get_transmitted_bits() 
// 							  << "  " << "suc pkt " << bus[b].alias_of_mt_container()[i].get_successful_packet()
// 							  << "  " << "drop pkt " << bus[b].alias_of_mt_container()[i].get_dropped_packet() << endl;
// 					}
// 				}  
// 				if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == 4)
// 				{
// 					if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 0 || bus[b].alias_of_mt_container()[i].get_traffic_type() == 1)
// 					{
// 						LTE << "i " << i << "  " << "h " << bus[b].alias_of_mt_container()[i].get_traffic_type() 
// 							<< "  " << "tx bit " << bus[b].alias_of_mt_container()[i].get_transmitted_bits() 
// 							<< "  " << "suc pkt " << bus[b].alias_of_mt_container()[i].get_successful_packet()
// 							<< "  " << "drop pkt " << bus[b].alias_of_mt_container()[i].get_dropped_packet() 
// 							<< "  " << "delay " << bus[b].alias_of_mt_container()[i].get_packet_delay() 	  
// 							<< "  " << "avg delay " << bus[b].alias_of_mt_container()[i].get_packet_delay() / bus[b].alias_of_mt_container()[i].get_successful_packet() << endl;			  
// 					}
// 					else
// 					{
// 						LTE << "i " << i << "  " << "h " << bus[b].alias_of_mt_container()[i].get_traffic_type() 
// 							<< "  " << "tx bit " << bus[b].alias_of_mt_container()[i].get_transmitted_bits() 
// 							<< "  " << "suc pkt " << bus[b].alias_of_mt_container()[i].get_successful_packet()
// 							<< "  " << "drop pkt " << bus[b].alias_of_mt_container()[i].get_dropped_packet() << endl;
// 					}
// 				}  
// 			}
// 			WCDMA << endl;
// 			LTE << endl;
//		}
/*********************************************************************************************************************************/

		/*--- 呼叫車子的mobility model讓車子移動 ---*/
		for(int b = 0; b < NUMBER_OF_BUS; b++)	
		{
			bus[b].mobility(); 
			/*--- 若bus到達終點 ---*/
			if(bus[b].get_bus_location() == bus[b].get_destination())
			{
				bus.erase(bus.begin() + b); //刪除此bus
				bus.push_back(Bus(b + 1, total_mt, 36, bs_location_container));//重新新增同編號的車子
			}
		}

/************************************** test:輸出車子每次移動完的座標及waypoint座標 ****************************************/
//// 		for(int b = 0; b < NUMBER_OF_BUS; b++)
//// 		{
////  				cout << "frame " << f << " total_mt " << total_mt << " seed " << seed 
//// 					 << " (" << bus[b].get_bus_location().first << ", " << bus[b].get_bus_location().second << ")" 
////  					 << " -> (" << bus[b].get_ctw().first << ", " << bus[b].get_ctw().second << ") " << endl;
//					<< "(" << bus[b].get_rtw().first << ", " << bus[b].get_rtw().second << ") "
// 				file1 << bus[b].get_bus_location().first << " " << bus[b].get_bus_location().second
// 					<< " -> (" << bus[b].get_ctw().first << ", " << bus[b].get_ctw().second << ")" << endl;
////		}
/*********************************************************************************************************************/
/********************************************* test:輸出車子的ho觸發前的網路,bs ********************************************/
//**** 		for(int b = 0; b < NUMBER_OF_BUS; b++)
// 		{
// 			cout << endl
// 				<< "serving bs set before ho_trigger(): " 
// 				<< bus[b].get_serving_bs_set()[0] << " " << bus[b].get_serving_bs_set()[1] << " " << bus[b].get_serving_bs_set()[2] << endl
// 				<< "old serving bs set before ho_trigger(): "
// 				<< bus[b].get_old_serving_bs_set()[0] << " " << bus[b].get_old_serving_bs_set()[1] << " " << bus[b].get_old_serving_bs_set()[2] << endl
// 				<< "old serving network before ho_trigger(): "
// 				<< bus[b].get_old_serving_network() << endl
// 				<< "-----------------------" << endl;
//**** 		}
/*********************************************************************************************************************/

		/*--- 呼叫車子的「換手觸發函數」---*/
		for(int b = 0; b <NUMBER_OF_BUS; b++)
			bus[b].ho_trigger();

/********************************************** test:輸出車子的ho觸發後的網路,bs *******************************************/
//		for(int b = 0; b < NUMBER_OF_BUS; b++)
//		{
//  			cout << endl
//  				 << "serving bs set after ho_trigger(): " 
//  				 << bus[b].get_serving_bs_set()[0] << " " << bus[b].get_serving_bs_set()[1] << " " << bus[b].get_serving_bs_set()[2] << endl
//  				 << "old serving bs set before ho_trigger(): "
// 				 << bus[b].get_old_serving_bs_set()[0] << " " << bus[b].get_old_serving_bs_set()[1] << " " << bus[b].get_old_serving_bs_set()[2] << endl
//  				 << "old serving network before ho_trigger(): "
//  				 << bus[b].get_old_serving_network() << endl
//  				 << "------------------------------------------------" << endl;
//			file3 << bus[b].get_serving_bs_set()[0] << " " << bus[b].get_serving_bs_set()[1] << " " << bus[b].get_serving_bs_set()[2] << endl;
			
			/************************ test:觀看flag *********************/
// 			cout << endl << "bus no. " << b+1 << " flag: " ;
//  			for(int j = 0; j < 3; j++)
//  				cout << bus[b].get_flag()[j] << " " ;
//  			cout << endl;
//  			cout << "---------------------" << endl;
// 		}
/*********************************************************************************************************************/

		/*--- check車子的flag容器,將容器裡等於true的網路的所有mt的index push到某容器準備換手 ---*/  
		gho_list.assign(NUMBER_OF_BUS, vector<vector<int> > (4, vector<int> (0) ));		//bus, traffic type, mt index

		for(int b = 0; b < NUMBER_OF_BUS; b++)								//第幾台車子(row)
		{	
			for(int j = 0; j < 3; j++)										//flag index   0(gsm/edge), 1(wcdma), 2(lte) 
			{
				if(j == 0)													//flag 0
				{
					if(bus[b].get_flag()[j] == true)											//true就會push mt進list
					{
						/*==========================================*/
						cout << "flag(GSM/EDGE, WCDMA, LTE): ";
						for(int jjj = 0; jjj < 3; jjj++)
							cout << bus[b].get_flag()[jjj] << " " ;
						/*==========================================*/

						bus[b].get_flag()[j] = false;											//flag歸零
						
						for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)
						{
							/////get_serving_bs_set再被呼叫前已被更新
							if( (bus[b].alias_of_mt_container()[i].get_network_bs_number().first == j + 1 &&    //gsm
								 bus[b].alias_of_mt_container()[i].get_network_bs_number().second == bus[b].get_old_serving_bs_set()[j])||
								(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == j + 2 &&	//edge
								 bus[b].alias_of_mt_container()[i].get_network_bs_number().second == bus[b].get_old_serving_bs_set()[j]) )
							{
								if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 0)				//vo
									gho_list[b][0].push_back(i);
								else if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 2)			//http
									gho_list[b][1].push_back(i);
								else if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 1)			//vi
									gho_list[b][2].push_back(i);
								else if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 3)			//ftp
									gho_list[b][3].push_back(i);

							}
							/*--- 把被駛離的cell的loading減掉 ---*/
 							if( bus[b].alias_of_mt_container()[i].get_network_bs_number().first == j + 1 &&    //gsm
								bus[b].alias_of_mt_container()[i].get_network_bs_number().second == bus[b].get_old_serving_bs_set()[j])
							{
//								cout << " g loading before " << g_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() << endl;
//								cout << " No. " << bus[b].alias_of_mt_container()[i].get_mt_number() << " h " << bus[b].alias_of_mt_container()[i].get_traffic_type()
//									 << " increased load " << g_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type()) << endl;
								g_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() -= g_increased_load(0);
//								cout << " g loading after " << g_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() << endl;
//								system("PAUSE");
							}
							else if(bus[b].alias_of_mt_container()[i].get_network_bs_number().first == j + 2 &&	//edge
								    bus[b].alias_of_mt_container()[i].get_network_bs_number().second == bus[b].get_old_serving_bs_set()[j])
							{
//								cout << " e loading before " << e_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() << endl;
//								cout << " No. " << bus[b].alias_of_mt_container()[i].get_mt_number() << " h " << bus[b].alias_of_mt_container()[i].get_traffic_type()
//									 << " increased load " << e_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type()) << endl;
								e_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() -= e_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type());
//								cout << " e loading after " << e_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() << endl;
//								system("PAUSE");
 							}
						}
					}
				}
				else    //flag j = 1(wcdma) or 2(lte)
				{	
					if(bus[b].get_flag()[j] == true)
					{
						bus[b].get_flag()[j] = false;											//flag歸零
						
						for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)
						{  
							if( (bus[b].alias_of_mt_container()[i].get_network_bs_number().first == j + 2 && 
								 bus[b].alias_of_mt_container()[i].get_network_bs_number().second == bus[b].get_old_serving_bs_set()[j]) )
							{
								if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 0)
									gho_list[b][0].push_back(i);
								else if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 2)
									gho_list[b][1].push_back(i);
								else if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 1)
									gho_list[b][2].push_back(i);
								else if(bus[b].alias_of_mt_container()[i].get_traffic_type() == 3) 
									gho_list[b][3].push_back(i);
							
								/*--- 把被駛離的cell的loading減掉 ---*/
 								if(j == 1) //wcdma
								{
//									cout << " w loading before " << w_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() << endl;
//									cout << " No. " << bus[b].alias_of_mt_container()[i].get_mt_number() << " h " << bus[b].alias_of_mt_container()[i].get_traffic_type()
//										 << " increased load " << w_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type()) << endl;
									w_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() -= w_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type());
//									cout << " w loading after " << w_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() << endl;
//									system("PAUSE");
								}
								else //j == 2(lte)
								{
//									cout << " l loading before " << l_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() << endl;
//									cout << " No. " << bus[b].alias_of_mt_container()[i].get_mt_number() << " h " << bus[b].alias_of_mt_container()[i].get_traffic_type()
//										 << " increased load " << l_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type()) << endl;
									l_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() -= l_increased_load(bus[b].alias_of_mt_container()[i].get_traffic_type());
//									cout << " l loading after " << l_bs[bus[b].get_old_serving_bs_set()[j] - 1].get_load_intensity() << endl;
//									system("PAUSE");
 								}
							}
						}
					}
				}
			}
		}

		/*--- group handover decision scheme ---*/
		for(int b = 0; b < NUMBER_OF_BUS; b++)
		{
			if( (gho_list[b][0].size() != 0)||(gho_list[b][1].size() != 0)||(gho_list[b][2].size() != 0) ||(gho_list[b][3].size() != 0) )	//有MT要做換手
			{	
				int num_of_blocked_mt = 0;

				int total_number_of_gho_mt = gho_list[b][0].size() + gho_list[b][1].size() + gho_list[b][2].size() + gho_list[b][3].size();
/************************************************************* print換手的人數 **************************************************************************/		
// 				cout << endl
// 					<< "gho list(vo h vi f): " 
// 					<< gho_list[b][0].size() << " " << gho_list[b][1].size() << " " << gho_list[b][2].size() << " " << gho_list[b][3].size()
// 					<< " " << "sum: " << gho_list[b][0].size() + gho_list[b][1].size() + gho_list[b][2].size() + gho_list[b][3].size() << endl
// 					<<"------------------------------------" << endl;
/********************************************************************************************************************************************/

/*********************************************************** 累積換手次數 **********************************************************/
				num_of_handover_in_system += (gho_list[b][0].size() + gho_list[b][1].size() + gho_list[b][2].size() + gho_list[b][3].size());
				NUM_OF_HO_IN_SYSTEM << f * 10e-3 << " " << num_of_handover_in_system << endl;
/*********************************************************************************************************************************/

				/*--- 計算在candidte bs set內的waypoint number及outer waypoint number ---*/
				double temp_dis[3] = {0, 0, 0};
				vector<vector<pair<double, double> > > big_omega_j(3, vector<pair<double, double> > (1, bus[b].get_bus_location())); //初始值為bus location
				vector<vector<int> > in_j_waypoint_number_set(3, vector<int> (0));
				unsigned int waypoint_number[3];
				int out_j_waypoint_number[3];

				for(int j = 0; j < 3; j++)
					waypoint_number[j] = bus[b].gctw_number();		//waypoint_number = 目前朝向的方向點編號
				
				for(int j = 0; j < 3; j++)
				{
					if(waypoint_number[j] < bus[b].get_total_point_set().size())
					{
						do 
						{
							//計算目前朝向方向點與BS距離
							temp_dis[j] = distance_between_points( bus[b].get_total_point_set()[waypoint_number[j]],
								bs_location_container[j][bus[b].get_serving_bs_set()[j] - 1] );
							//若距離小於半徑(此方向點在圓內)
							if(temp_dis[j] < radius[j])
							{
								//把此方向點加入集合
								big_omega_j[j].push_back(bus[b].get_total_point_set()[waypoint_number[j]]);		//溢位bingo
								in_j_waypoint_number_set[j].push_back(waypoint_number[j]);
								//把目前朝向的方向點更新成下一個方向點
								waypoint_number[j]++;
								//若新的方向點的編號大於終點的編號
								if(waypoint_number[j] == bus[b].get_total_point_set().size())	
								{
									out_j_waypoint_number[j] = -1;
									break;
								}
							}
							//若距離大於半徑(此方向點在圓外)
							else
							{
								out_j_waypoint_number[j] = waypoint_number[j];
							}
						} while (temp_dis[j] < radius[j]);
					}	
				}

				/*--- dwelling time ---*/
				double p1[3] = {0, 0, 0}, p[3] = {0, 0, 0};
				double dwelling_time[3] = {0, 0, 0};

				for(int j = 0; j < 3; j++)
				{
					if(big_omega_j[j].size() == 1)
					{
//						bus.alias_of_mt_container()[list[h][i]].get_life_time()

						double temp_d = bus[b].distance_between_points(big_omega_j[j].front(), bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j] - 1]);
						double temp_theta = bus[b].angle_between_vectors(make_pair(bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j]-1].first - big_omega_j[j].front().first, bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j]-1].second - big_omega_j[j].front().second),
							make_pair(bus[b].get_total_point_set()[out_j_waypoint_number[j]].first - big_omega_j[j].front().first, bus[b].get_total_point_set()[out_j_waypoint_number[j]].second - big_omega_j[j].front().second));
						p1[j] = temp_d * cos(temp_theta * PI / 180) + sqrt( pow(radius[j], 2) - pow(temp_d * sin(temp_theta * PI / 180), 2) );

						//test
// 						cout << "big_omega_j[j].front(): (" << big_omega_j[j].front().first << ", " << big_omega_j[j].front().second  << ")" << endl;
// 						cout << "bs location: (" <<  bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j] - 1].first << ", " << bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j] - 1].second << ")" << endl;
// 						cout << "(size = 1)temp_d: " << temp_d << " temp_theta: " << temp_theta << endl;
// 						cout << "d * cos(theta): " << temp_d * cos(temp_theta * PI / 180) << " sqrt: " << sqrt( pow(radius[j], 2) - pow(temp_d * sin(temp_theta * PI / 180), 2) ) << " p1[j]: " << p1[j] << endl;

						dwelling_time[j] = p1[j] / (bus[b].get_speed() / 3600); 
					}
					else if(big_omega_j[j].size() > 1)
					{
						if(big_omega_j[j].back() == bus[b].get_destination())
						{
							p[j] = 0;
						}
						else
						{
							double temp_d = bus[b].distance_between_points(big_omega_j[j].back(), bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j] - 1]);
							double temp_theta = bus[b].angle_between_vectors(make_pair(bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j]-1].first - big_omega_j[j].back().first, bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j] - 1].second - big_omega_j[j].back().second),
								make_pair(bus[b].get_total_point_set()[out_j_waypoint_number[j]].first - big_omega_j[j].back().first, bus[b].get_total_point_set()[out_j_waypoint_number[j]].second - big_omega_j[j].back().second));
							p[j] = temp_d * cos(temp_theta * PI / 180) + sqrt( pow(radius[j], 2) - pow(temp_d * sin(temp_theta * PI / 180), 2) );

							//test
// 							cout << "big_omega_j[j].front: (" << big_omega_j[j].front().first << ", " << big_omega_j[j].front().second << ")" << endl;
// 							cout << "bs location: (" <<  bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j] - 1].first << ", " << bus[b].get_bs_location()[j][bus[b].get_serving_bs_set()[j] - 1].second << ")" << endl;
// 							cout << "(size > 1)temp_d: " << temp_d << " temp_theta: " << temp_theta << endl;
// 							cout << "d*cos(theta): " << temp_d * cos(temp_theta * PI / 180) << " sqrt: " << sqrt( pow(radius[j], 2) - pow(temp_d * sin(temp_theta * PI / 180), 2) ) << " p[j]: " << p[j] << endl;
						}

						double dis_sum = 0;
						double wtime_sum = 0;
						for(unsigned int a = 0; a < big_omega_j[j].size() - 1; a++)
						{
							dis_sum += bus[b].distance_between_points(big_omega_j[j][a], big_omega_j[j][a + 1]);
							wtime_sum += bus[b].get_waiting_time_set()[in_j_waypoint_number_set[j][a]];
						}
						dwelling_time[j] = ((p[j] + dis_sum) / (bus[b].get_speed() / 3600)) + wtime_sum;
					}
				}
				/*--- dwelling time end ---*/

/**************************************** test:big_omega_j內容與out_j_waypoint_number ******************************************/ 						
// 				for(int j = 0; j < 3; j++)
// 				{	
// 					cout << "big omega " << j << ": " ;
// 					for(unsigned int w = 0; w < big_omega_j[j].size(); w++)
// 						cout << "(" << big_omega_j[j][w].first << " " << big_omega_j[j][w].second << ") ";
// 					cout << endl << "in " << j << " waypoint_number_set: " ;
// 					for(unsigned int w = 0; w < in_j_waypoint_number_set[j].size(); w++)
// 						cout << in_j_waypoint_number_set[j][w] << " ";
// 					cout << endl << "out "<< j << " waypoint_number: " << out_j_waypoint_number[j] << endl; 	
// 				}
// 				system("PAUSE");
/******************************************************************************************************************************/
				
				/*--- for QoS factor & joinable network constraint ---*/ //voice video 分開計算
				vector<vector<double> > network_avg_delay;
				network_avg_delay.assign(4, vector<double> (2, 0));
				//(voice)		
				//(video)
				//(voice)(video)
				//(voice)(video)
				if(g_bs[bus[b].get_serving_bs_set()[0] - 1].get_gsm_successful_packet() != 0 )		//gsm
					network_avg_delay[0][0] = g_bs[bus[b].get_serving_bs_set()[0] - 1].get_gsm_packet_delay() / g_bs[bus[b].get_serving_bs_set()[0] - 1].get_gsm_successful_packet();
				else
					network_avg_delay[0][0] = 0;	
				if(e_bs[bus[b].get_serving_bs_set()[0] - 1].get_edge_successful_packet()[0] != 0)	//edge	
					network_avg_delay[1][0] = e_bs[bus[b].get_serving_bs_set()[0] - 1].get_edge_packet_delay()[0] / e_bs[bus[b].get_serving_bs_set()[0] - 1].get_edge_successful_packet()[0];
				else
					network_avg_delay[1][0] = 0;
				if(w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_successful_packet()[0] != 0)	//wcdma
					network_avg_delay[2][0] = w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_packet_delay()[0] / w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_successful_packet()[0];
				else
					network_avg_delay[2][0] = 0;
				if(w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_successful_packet()[1] != 0)	//wcdma
					network_avg_delay[2][1] = w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_packet_delay()[1] / w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_successful_packet()[1];
				else
					network_avg_delay[2][1] = 0;
				if(l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_successful_packet()[0] != 0)	//lte
					network_avg_delay[3][0] = l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_packet_delay()[0] / l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_successful_packet()[0];
				else
					network_avg_delay[3][0] = 0;
				if(l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_successful_packet()[1] != 0)	//lte
					network_avg_delay[3][1] = l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_packet_delay()[1] / l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_successful_packet()[1];
				else
					network_avg_delay[3][1] = 0;

// 				//************test
// 				for(int j = 0; j < 4; j++)
// 				{
// 					for(int h = 0; h < 2 ; h++)
// 					{
// 						cout << "network_avg_delay[" << j << "][" << h << "] = " << network_avg_delay[j][h] << endl;
// 					}
// 				}

				vector<vector<double> > network_avg_drop_rate;
				network_avg_drop_rate.assign(4, vector<double> (2, 0));
				//gsm
				if(g_bs[bus[b].get_serving_bs_set()[0] - 1].get_gsm_successful_packet() != 0)
					network_avg_drop_rate[0][0] = (double)dropped_packet[0][bus[b].get_serving_bs_set()[0] - 1][0] / (dropped_packet[0][bus[b].get_serving_bs_set()[0] - 1][0] + g_bs[bus[b].get_serving_bs_set()[0] - 1].get_gsm_successful_packet());
				else
					network_avg_drop_rate[0][0] = 0;
				//edge
				if(e_bs[bus[b].get_serving_bs_set()[0] - 1].get_edge_successful_packet()[0] != 0)
					network_avg_drop_rate[1][0] = (double)dropped_packet[1][bus[b].get_serving_bs_set()[0] - 1][1] / (dropped_packet[1][bus[b].get_serving_bs_set()[0] - 1][1] + e_bs[bus[b].get_serving_bs_set()[0] - 1].get_edge_successful_packet()[0]);
				else
					network_avg_drop_rate[1][0] = 0;
				//wcdma
				if(w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_successful_packet()[0] != 0)
					network_avg_drop_rate[2][0] = (double)dropped_packet[2][bus[b].get_serving_bs_set()[1] - 1][0] / (dropped_packet[2][bus[b].get_serving_bs_set()[1] - 1][0] + w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_successful_packet()[0]);
				else
					network_avg_drop_rate[2][0] = 0;	
				if(w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_successful_packet()[1] != 0)
					network_avg_drop_rate[2][1] = (double)dropped_packet[2][bus[b].get_serving_bs_set()[1] - 1][1] / (dropped_packet[2][bus[b].get_serving_bs_set()[1] - 1][1] + w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_successful_packet()[1]);
				else
					network_avg_drop_rate[2][1] = 0;
				//lte
				if(l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_successful_packet()[0] != 0)
					network_avg_drop_rate[3][0] = (double)dropped_packet[3][bus[b].get_serving_bs_set()[2] - 1][0] / (dropped_packet[3][bus[b].get_serving_bs_set()[2] - 1][0] + l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_successful_packet()[0]);
				else
					network_avg_drop_rate[3][0] = 0;
				if(l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_successful_packet()[1] != 0)
					network_avg_drop_rate[3][1] = (double)dropped_packet[3][bus[b].get_serving_bs_set()[2] - 1][1] / (dropped_packet[3][bus[b].get_serving_bs_set()[2] - 1][1] + l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_successful_packet()[1]);
				else
					network_avg_drop_rate[3][1] = 0;

// 				//********test
// 				for(int j = 0; j < 4; j++)
// 				{
// 					for(int h = 0; h < 2 ; h++)
// 					{
// 						cout << "network_avg_drop_rate[" << j << "][" << h << "] = " << network_avg_drop_rate[j][h] << endl; 
// 					}
// 				}

				vector<vector<double> > network_avg_trans_rate;
				network_avg_trans_rate.assign(3, vector<double> (2, 0));
				if(f != 0)
				{
					//edge
					network_avg_trans_rate[0][0] = e_bs[bus[b].get_serving_bs_set()[0] - 1].get_edge_transmitted_bit()[1] / (f * 10e-3);
					network_avg_trans_rate[0][1] = e_bs[bus[b].get_serving_bs_set()[0] - 1].get_edge_transmitted_bit()[2] / (f * 10e-3); 
					//wcdma
					network_avg_trans_rate[1][0] = w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_transmitted_bit()[2] / (f * 10e-3);
					network_avg_trans_rate[1][1] = w_bs[bus[b].get_serving_bs_set()[1] - 1].get_wcdma_transmitted_bit()[3] / (f * 10e-3);
					//lte
					network_avg_trans_rate[2][0] = l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_transmitted_bit()[2] / (f * 10e-3);
					network_avg_trans_rate[2][1] = l_bs[bus[b].get_serving_bs_set()[2] - 1].get_lte_transmitted_bit()[3] / (f * 10e-3);
				}
				
// 				//********test
// 				for(int j = 0; j < 3; j++)
// 				{
// 					for(int h = 0; h < 2 ; h++)
// 					{
// 						cout << "network_avg_trans_rate[" << j << "][" << h << "] = " << network_avg_trans_rate[j][h] << endl;
// 					}
// 				}
// 
// 				system("PAUSE");

				//----------------------------- sa_scheme ----------------------------------------
				sa_scheme(bus[b], gho_list[b], f, dwelling_time,
					      g_bs[bus[b].get_serving_bs_set()[0] - 1].get_load_intensity(),
						  e_bs[bus[b].get_serving_bs_set()[0] - 1].get_load_intensity(),
						  w_bs[bus[b].get_serving_bs_set()[1] - 1].get_load_intensity(),
						  l_bs[bus[b].get_serving_bs_set()[2] - 1].get_load_intensity(),
						  network_avg_delay, network_avg_drop_rate, network_avg_trans_rate,
						  num_of_blocked_mt);												//get_serving_bs_set為最新的值
				
				cumulated_times_of_gho ++;
				GHO_BLOACKING_RATIO << f * 10e-3 << " " << cumulated_times_of_gho << " " << (double)num_of_blocked_mt / total_number_of_gho_mt << endl;
				sum_of_gho_blocking_ratio += (double)num_of_blocked_mt / total_number_of_gho_mt;
				AVERAGE_GHO_BLOACKING_RATIO << (double)sum_of_gho_blocking_ratio / cumulated_times_of_gho << endl;

				//debug
				cout << "cumulated_times_of_gho " << cumulated_times_of_gho << endl;
				cout << "num_of_blocked_mt " << num_of_blocked_mt << endl;
				cout << "num_of_blocked_mt / total_number_of_gho_mt " << (double)num_of_blocked_mt / total_number_of_gho_mt << endl;
				cout << "sum_of_gho_blocking_ratio " << sum_of_gho_blocking_ratio << endl;
				cout << "sum_of_gho_blocking_ratio / cumulated_times_of_gho " << (double)sum_of_gho_blocking_ratio / cumulated_times_of_gho << endl;

				/*--- 計算所有bs的load intensity factor---*/
				double average_load_intensity = 0, sum_of_load_intensity = 0, sum_of_square_deviation = 0, load_balance_factor = 0; 
				for(int k = 0; k < NUMBER_OF_GSM_BS; k++)
					sum_of_load_intensity += g_bs[k].get_load_intensity(); 
				for(int k = 0; k < NUMBER_OF_EDGE_BS; k++)
					sum_of_load_intensity += e_bs[k].get_load_intensity(); 
				for(int k = 0; k < NUMBER_OF_WCDMA_BS; k++)
					sum_of_load_intensity += w_bs[k] .get_load_intensity(); 
				for(int k = 0; k < NUMBER_OF_LTE_BS; k++)
					sum_of_load_intensity += l_bs[k].get_load_intensity(); 
				average_load_intensity = sum_of_load_intensity / (NUMBER_OF_GSM_BS + NUMBER_OF_EDGE_BS + NUMBER_OF_WCDMA_BS + NUMBER_OF_LTE_BS);
				
				for(int k = 0; k < NUMBER_OF_GSM_BS; k++)
					sum_of_square_deviation += pow(g_bs[k].get_load_intensity() - average_load_intensity, 2);
				for(int k = 0; k < NUMBER_OF_EDGE_BS; k++)
					sum_of_square_deviation += pow(e_bs[k].get_load_intensity() - average_load_intensity, 2);
				for(int k = 0; k < NUMBER_OF_WCDMA_BS; k++)
					sum_of_square_deviation += pow(w_bs[k].get_load_intensity() - average_load_intensity, 2);
				for(int k = 0; k < NUMBER_OF_LTE_BS; k++)
					sum_of_square_deviation += pow(l_bs[k].get_load_intensity() - average_load_intensity, 2);
				load_balance_factor = sqrt(sum_of_square_deviation / (NUMBER_OF_GSM_BS + NUMBER_OF_EDGE_BS + NUMBER_OF_WCDMA_BS + NUMBER_OF_LTE_BS));
				LOAD_BALANCING_FACTOR_OF_ALL_BSS << f * 10e-3 << " " << cumulated_times_of_gho << " " << load_balance_factor << endl;
				sum_of_load_balancing_factor += load_balance_factor;
				AVERAGE_LOAD_BALANCING_FACTOR_OF_ALL_BSS << sum_of_load_balancing_factor / cumulated_times_of_gho << endl;
			}
		}

		/*--- 計算所有bs的average load intensity ---*/
// 		if(f % 100 == 0)		//每隔一秒記錄一次
// 		{
// 			double average_load_intensity = 0, sum_of_load_intensity = 0;
// 			for(int k = 0; k < NUMBER_OF_GSM_BS; k++)
// 				sum_of_load_intensity += g_bs[k].get_load_intensity(); 
// 			for(int k = 0; k < NUMBER_OF_EDGE_BS; k++)
// 				sum_of_load_intensity += e_bs[k].get_load_intensity(); 
// 			for(int k = 0; k < NUMBER_OF_WCDMA_BS; k++)
// 				sum_of_load_intensity += w_bs[k].get_load_intensity(); 
// 			for(int k = 0; k < NUMBER_OF_LTE_BS; k++)
// 				sum_of_load_intensity += l_bs[k].get_load_intensity(); 
// 			average_load_intensity = sum_of_load_intensity / (NUMBER_OF_GSM_BS + NUMBER_OF_EDGE_BS + NUMBER_OF_WCDMA_BS + NUMBER_OF_LTE_BS);
// 			sum_of_average_load_intensity += average_load_intensity;
// 
// 			AVERAGE_LOAD_INTENSITY_OF_ALL_BSS << f * 10e-3 << " " << average_load_intensity << endl;
// 			AVERAGE_LOAD_INTENSITY_OF_ALL_BSS_SUM_OVER_TIME << f * 10e-3 << " " << sum_of_average_load_intensity / ((f + 100) * 10e-3) << endl;
// 		}

		/*--- life time 減少 ---*/
		for(int b = 0; b < NUMBER_OF_BUS; b++)
		{
			for(unsigned int i = 0; i < bus[b].alias_of_mt_container().size(); i++)
			{
				if(bus[b].alias_of_mt_container()[i].get_life_time() > 0)
					bus[b].alias_of_mt_container()[i].get_life_time() -= 10e-3;
				else
					bus[b].alias_of_mt_container()[i].get_network_bs_number().first = 0;
			}
		}

		/*--- 暫時不改變fixed mt的life time ---*/
// 		for(int B = 0; B < NUMBER_OF_GSM_BS; B++)
// 		{
// 			for(unsigned int i = 0; i < g_bs[B].get_gsm_fixed_mt().size(); i++)
// 			{
// 				if(g_bs[B].get_gsm_fixed_mt()[i].get_life_time() > 0)
// 					g_bs[B].get_gsm_fixed_mt()[i].get_life_time() -= 10e-3;
// 				else
// 					g_bs[B].get_gsm_fixed_mt()[i].get_network_bs_number().first = 0;
// 			}
// 		}

	}

	/*--- data saving ---*/
	for(int B = 0; B < NUMBER_OF_GSM_BS; B++)
	{
		sum_of_voice_packet_delay += g_bs[B].get_gsm_packet_delay();
		sum_of_voice_suc_packet += g_bs[B].get_gsm_successful_packet();
		sum_of_voice_drop_packet += dropped_packet[0][B][0];
		sum_of_trans_bit += g_bs[B].get_gsm_transmitted_bit();
	}
	for(int B = 0; B < NUMBER_OF_EDGE_BS; B++)
	{
		sum_of_video_packet_delay += e_bs[B].get_edge_packet_delay()[0];
		sum_of_video_suc_packet += e_bs[B].get_edge_successful_packet()[0];
		sum_of_video_drop_packet += dropped_packet[1][B][1];
		sum_of_trans_bit += (e_bs[B].get_edge_transmitted_bit()[0] + e_bs[B].get_edge_transmitted_bit()[1] + e_bs[B].get_edge_transmitted_bit()[2]);
	}
	for(int B = 0; B < NUMBER_OF_WCDMA_BS; B++)
	{
		sum_of_voice_packet_delay += w_bs[B].get_wcdma_packet_delay()[0];
		sum_of_video_packet_delay += w_bs[B].get_wcdma_packet_delay()[1];
		sum_of_voice_suc_packet += w_bs[B].get_wcdma_successful_packet()[0];
		sum_of_video_suc_packet += w_bs[B].get_wcdma_successful_packet()[1];
		sum_of_voice_drop_packet += dropped_packet[2][B][0];
		sum_of_video_drop_packet += dropped_packet[2][B][1];
		sum_of_trans_bit += (w_bs[B].get_wcdma_transmitted_bit()[0] + w_bs[B].get_wcdma_transmitted_bit()[1] + w_bs[B].get_wcdma_transmitted_bit()[2] + w_bs[B].get_wcdma_transmitted_bit()[3]);
	}
	for(int B = 0; B < NUMBER_OF_LTE_BS; B++)
	{
		sum_of_voice_packet_delay += l_bs[B].get_lte_packet_delay()[0];
		sum_of_video_packet_delay += l_bs[B].get_lte_packet_delay()[1];
		sum_of_voice_suc_packet += l_bs[B].get_lte_successful_packet()[0];
		sum_of_video_suc_packet += l_bs[B].get_lte_successful_packet()[1];
		sum_of_voice_drop_packet += dropped_packet[3][B][0];
		sum_of_video_drop_packet += dropped_packet[3][B][1];
		sum_of_trans_bit += (l_bs[B].get_lte_transmitted_bit()[0] + l_bs[B].get_lte_transmitted_bit()[1] + l_bs[B].get_lte_transmitted_bit()[2] + l_bs[B].get_lte_transmitted_bit()[3]);
	}

	AVG_VOICE_PKT_DELAY << total_mt << " " << (sum_of_voice_packet_delay / sum_of_voice_suc_packet) << endl;
	AVG_VIDEO_PKT_DELAY << total_mt << " " << (sum_of_video_packet_delay / sum_of_video_suc_packet) << endl;
	AVG_VOICE_DROP_RATE << total_mt << " " << (double) sum_of_voice_drop_packet / (sum_of_voice_drop_packet + sum_of_voice_suc_packet) << endl;
	AVG_VIDEO_DROP_RATE << total_mt << " " << (double) sum_of_video_drop_packet / (sum_of_video_drop_packet + sum_of_video_suc_packet) << endl;
	THROUGHPUT << total_mt << " " << sum_of_trans_bit / (f * 10e-3) << endl;

//	file1.close();
// 	file2.close();
// 	file3.close();
// 	file4.close();
// 	GSM.close();
// 	EDGE.close();
// 	WCDMA.close();
// 	LTE.close();
	NUM_OF_HO_IN_SYSTEM.close();
	LOAD_BALANCING_FACTOR_OF_ALL_BSS.close();
	GHO_BLOACKING_RATIO.close();
	AVERAGE_GHO_BLOACKING_RATIO.close();
	AVG_VOICE_PKT_DELAY.close();
	AVG_VIDEO_PKT_DELAY.close();
	AVG_VOICE_DROP_RATE.close();
	AVG_VIDEO_DROP_RATE.close();
	THROUGHPUT.close();
	AVERAGE_LOAD_BALANCING_FACTOR_OF_ALL_BSS.close();
    }
   }
  }
  return 0;
}
/*--- 兩點之間的距離 ---*/
double distance_between_points(const pair<double, double> &i, const pair<double, double> &j)
{
	return hypot(i.first - j.first, i.second - j.second);			//回傳兩個參數平方和的平方根，也就是直角三角形的斜邊長。
}


// double dwelling_time(double radius, pair<double, double> bs_position, pair<double, double> bus_position, double bus_speed, vector<pair<double, double>> waypoints, pair<double, double> outer_waypoint)
// {
// 
// 	if(waypoints.size() == 0)
// 	{
// 		double beta;
// 		double gama = radius;
// 		double theta;
// 		double p_total;
// 		pair<double, double> v_a, v_b;
// 		double norm_a, norm_b, dot_product;
// 		beta = distance_between_points(bs_position, bus_position); 
// 		
// 		v_a = make_pair(bs_position.first - bus_position.first, bs_position.second - bus_position.second);
// 		v_b = make_pair(waypoints[0].first - bus_position.first, waypoints[0].second - bus_position.second);
// 		norm_a = hypot(v_a.first, v_a.second);
// 		norm_b = hypot(v_b.first, v_b.second);
// 		dot_product = v_a.first *v_b.first + v_a.second * v_b.second;
// 		theta = acos(dot_product / (norm_a * norm_b));
// 		
// 		p_total = beta * cos(theta) + sqrt(pow(gama, 2) + pow(beta * sin(theta), 2));
// 		return p_total;
// 	}
// 	else
// 	{		
// 		double beta;
// 		double gama = radius;
// 		double theta;
// 		double p_total;
// 		double pa_total = 0;
// 		pair<double, double> v_a, v_b;
// 		double norm_a, norm_b, dot_product;
// 		beta = distance_between_points(bs_position, waypoints.back()); 
// 		
// 		v_a = make_pair(bs_position.first - waypoints.back().first, bs_position.second - waypoints.back().second);
// 		v_b = make_pair(outer_waypoint.first - waypoints.back().first, outer_waypoint.second - waypoints.back().second);
// 		norm_a = hypot(v_a.first, v_a.second);
// 		norm_b = hypot(v_b.first, v_b.second);
// 		dot_product = v_a.first *v_b.first + v_a.second * v_b.second;
// 		theta = acos(dot_product / (norm_a * norm_b));					
// 		
// 		double p0 = distance_between_points(bus_position, waypoints[0]);
// 		for(unsigned int i = 0; i < waypoints.size()-1; i++)								//waypoints.size() = 1會不會出錯?
// 		{
// 			pa_total = pa_total + distance_between_points(waypoints[i], waypoints[i+1]);
// 		}
// 		p_total = p0 + pa_total + beta * cos(theta) + sqrt(pow(gama, 2) + pow(beta * sin(theta), 2));
// 		return p_total;
// 	}
// }


/********throughput*******/
//傳輸的bit數/模擬時間(有傳+沒傳)
/****transmission rate****/
//傳輸的bit數/有在傳輸的時間