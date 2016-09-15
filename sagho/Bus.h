#ifndef BUS_H_
#define BUS_H_

#include <vector>
#include <utility>
#include <iostream>
#include <cmath>
#include "Mt.h"
#include "path_loss.h"
#include "distributions.h"
#include <sstream>  //ostringstream
#include <fstream>	//ofstream
#include <algorithm>

using namespace std;
//#define NUMBER_OF_WAYPOINT 100

class Bus
{
private:
	/*--- 引數 ---*/
	int bus_number;											//車子的編號
	int num_of_mt;											//車內的總人數
	double speed;											//車子的速度
	vector<vector<pair<double, double>>> bs_location;	

	/*--- 不會改變值的參數 ---*/
	static const int num_of_bus;							//車子的數量
	double moving_distance;									//車子每frame移動的距離
	int num_of_each_type_mt[4];								//0vo 1vi 2http 3ftp 
	
	/*--- 會改變值的參數 ---*/	
	pair<double,double> bus_location;						//車子的位置
	pair<double, double> source, destination;				//方向點
	vector<pair<double,double>> total_point_set, reverse_point_set;
	vector<double> total_angle_set, temp_angle_set, reverse_angle_set;
	vector<double> waiting_time_set, reverse_waiting_time_set;	
	double path, theta, alpha, phi, psi;
	int idx;
	pair<double, double> created_waypoint, center;

	pair<double, double> initial_point, end_point, temp_point;

	bool flag[3];
	int cumulate_num_of_handover;

	int serving_bs_set[3];
	int old_serving_bs_set[3];
	int old_serving_network;

	double angle;											//車子座標和方向點形成的向量與單位向量(1,0)的夾角
	double distance;										//車子座標和方向點的距離
	double wtime_of_ctw, rwtime_of_rtw;						//目前朝向的方向點的等待時間
	int wtime_number_of_ctw;								//等待時間編號 current waiting time number	of current toward waypoint
	int rwtime_number_of_rtw;	
	int ctw_number, rtw_number;								//current toward waypoint number, reverse current toward waypoint number
	int tas_number, ras_number;								//total_angle_set_number, reverse_angle_set_number;
	pair<double, double> ctw, rtw;							//current toward waypoint	

	/*--- 存資料的容器 ---*/
	vector<Mt> mt_container;							    //車子裡用來儲存mt的容器

public:
	Bus();
	Bus(int set_bus_number, int set_num_of_mt, double set_speed, 
		vector<vector<pair<double, double> > > get_bs_location_container);
	~Bus();
	pair<double, double> get_bus_location();					//取用車子座標位置
	vector<vector<pair<double, double> > > get_bs_location();
	double get_speed();											//取用車子速度
	pair<double, double> get_ctw(), get_rtw();					//取用方向點
	pair<double, double> get_source(), get_destination();
	int gctw_number();											//current toward waypoint number
	vector<pair<double, double> > &get_total_point_set();
	vector<double> get_waiting_time_set();
	double get_waiting_time();									//取用等待時間
	bool *get_flag();
	int get_cumulate_num_of_handover();
	int *get_serving_bs_set();
	int *get_old_serving_bs_set();
	int get_old_serving_network();

	int get_num_of_mt();

	void mobility();											//車子每單位frame的移動方式
	void ho_trigger();

	double distance_between_points(const pair<double, double> &, const pair<double, double> &);	//兩點之間的距離
	double angle_between_points(const pair<double, double> &, const pair<double, double> &);	//兩點之間的夾角
	double angle_between_vectors(const pair<double, double> &, const pair<double, double> &);	//兩向量之間的夾角

	vector<Mt> &alias_of_mt_container();

};

#endif