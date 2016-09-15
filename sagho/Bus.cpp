//#include <iostream>
//#include <cmath>
//#include <vector>
//#include "Mt.h"
//#include "distributions.h"
//#include "path_loss.h"
#include "Bus.h"

/*--- 預設建構子 ---*/
Bus::Bus()
{
	bus_number = -1;
	num_of_mt = -1;
	speed = -1;
	bs_location.assign(3, vector<pair<double, double>> (27, make_pair(-1.0, -1.0)));

	moving_distance = speed /3600 * 10e-3;	

	path = 1.1;
	theta = unifrnd(0.0, 2 * PI);
	// 	source = make_pair(0.0, 1.1);
	// 	destination = make_pair(0.0, -1.1);

	source.first = path * cos(theta);			//generate initial point of a bus
	source.second = path * sin(theta);
	bus_location = source;
	destination.first = (0.0 - source.first);	//generate destination point of the bus
	destination.second = (0.0 - source.second);
	total_point_set.push_back(source);			//total_point_set現在有一個元素source

	alpha = -1;
	phi = -1;
	psi = -1;
	idx = 0;
	created_waypoint = make_pair(1000, 1000);
	center = make_pair(0.0, 0.0);

	while(total_point_set[idx] != destination)
	{
		do{

			alpha = unifrnd(0.2, 0.3);
			phi = unifrnd(-60, 60);

			if(destination.second - total_point_set[idx].second >= 0)
				psi = angle_between_vectors(make_pair((destination.first - total_point_set[idx].first), (destination.second - total_point_set[idx].second)), make_pair(1, 0));
			else
				psi = 360 - angle_between_vectors(make_pair((destination.first - total_point_set[idx].first), (destination.second - total_point_set[idx].second)), make_pair(1, 0));							
			created_waypoint = make_pair(total_point_set[idx].first + alpha * cos((psi + phi) * PI / 180), total_point_set[idx].second + alpha * sin((psi + phi) * PI / 180));	

// 			cout  << idx << " " << "psi: " << psi << " phi:" << phi << "psi + phi:" << psi+phi << endl; 
// 			cout << "x" << alpha * cos((psi + phi) * PI / 180) << " " << "y " << alpha * sin((psi + phi) * PI / 180) << endl; 

		}while(distance_between_points(created_waypoint, center) >= 1.1);

		double d_temp = distance_between_points(total_point_set[idx], destination);

		if(d_temp > 0.3)
		{
			total_point_set.push_back(created_waypoint);
			total_angle_set.push_back( (psi + phi) * PI / 180  );						//注意!單位已換為徑度 
			temp_angle_set.push_back( (psi + phi - 180) * PI / 180   );					//注意!單位已換為徑度   
		}
		else
		{
			total_point_set.push_back(destination);
			total_angle_set.push_back(psi * PI / 180);									//注意!單位已換為徑度 
			temp_angle_set.push_back( (psi - 180) * PI / 180  );						//注意!單位已換為徑度  
		}

		idx++;
	}

	reverse_angle_set.resize(temp_angle_set.size(), 0);
	reverse_copy(temp_angle_set.begin(), temp_angle_set.end(), reverse_angle_set.begin());

	for(unsigned int n = 0; n < total_point_set.size(); n++)
		waiting_time_set.push_back(unifrnd(0.0, 0.5));

	reverse_waiting_time_set.resize(waiting_time_set.size(), 0);
	reverse_copy(waiting_time_set.begin(), waiting_time_set.end(), reverse_waiting_time_set.begin());

	reverse_point_set.resize(total_point_set.size(), make_pair(0,0));
	reverse_copy(total_point_set.begin(), total_point_set.end(), reverse_point_set.begin());

	// 	reverse_order_of_tps.reserve(total_point_set.size());
	// 	for(int m = total_point_set.size() - 1; m >= 0; m--)
	// 		reverse_order_of_tps.push_back(total_point_set[m]);

	/*----------------------test by yi-shing---------------------------------------*/
// 	cout << total_point_set.size() << " " << reverse_point_set.size() << endl;
// 	cout << total_angle_set.size() << " " << reverse_angle_set.size() << endl;
// 	cout << total_point_set.back().first << " " << total_point_set.back().second << endl;
// 	cout << reverse_point_set.back().first << " " << reverse_point_set.back().second << endl;
// 	for(unsigned int i = 0; i < total_angle_set.size(); i++)
// 		cout << total_angle_set[i] << endl;
// 	cout << endl << endl;
// 	for(unsigned int i = 0; i < temp_angle_set.size(); i++)
// 		cout << temp_angle_set[i] << endl;
// 	cout << endl << endl;
// 	for(unsigned int i = 0; i < reverse_angle_set.size(); i++)
// 		cout << reverse_angle_set[i] << endl;
// 	system("PAUSE"); 

	/*-----------------------------------------------------------------------------------------------------------------------------------*/
	/*test the correction of all points*/
	// 	for(unsigned int i = 0; i < total_point_set.size(); i++)
	// 	{
	// 		cout << total_point_set[i].first << " " << total_point_set[i].second << " "
	// 		     << reverse_order_of_tps[total_point_set.size() - i - 1].first << " " << reverse_order_of_tps[total_point_set.size() - i - 1].second << endl;
	// 	}
	// 	for(int i = (int)reverse_order_of_tps.size()-1; i >= 0 ; i--)												//用unsigned會溢位出錯	
	// 		cout << reverse_order_of_tps[i].first << " " << reverse_order_of_tps[i].second << endl;
	//	cout << system("PAUSE");

	/*------------------------------ mobility model用 ------------------------------*/
	ctw = total_point_set[1];										//初始化目前朝向的方向點
	ctw_number = 1;													//初始化目前朝向的方向點編號
	rtw = reverse_point_set[1];										//初始化回程時的朝向的方向點					
	rtw_number = 1;													//初始化回程時的朝向的方向點編號
	wtime_of_ctw = waiting_time_set[0];								//初始停留時間	
	wtime_number_of_ctw = 0;										//初始停留時間編號
	rwtime_of_rtw = reverse_waiting_time_set[0];
	rwtime_number_of_rtw = 0;
	tas_number = 0;													//初始角度編號
	ras_number = 0;													//初始回程角度編號
	initial_point = source;											//起點			
	end_point = destination;										//終點			
	//--------------------------------------------------------------------------------

	for(int j = 0;j < 3; j++)
	{
		flag[j] = false;				//判斷是否換手
		serving_bs_set[j] = -1;			//各個網路裡訊號最強的bs	
		old_serving_bs_set[j] = -1;
	}
	
	old_serving_network = -1;
	cumulate_num_of_handover = 0;

	int cumu_i = 0;
	for(int h = 0; h < 4; h++)
	{	
		for(int i = 0; i < num_of_each_type_mt[h]; i++)
		{
			mt_container.push_back(Mt(cumu_i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
			cumu_i++;
		}
	}

// 	for(int i = 0; i < num_of_mt; i++)
// 	{
// 		int h = rand()%4;				//h = 0, 1, 2, 3
// 		mt_container.push_back(Mt(i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
// 		//		alias_of_mt_container().push_back(Mt(i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
// 	}
}

/*--- 建構子 ---*/
Bus::Bus(int set_bus_number, int set_num_of_mt, double set_speed, 
	     vector<vector<pair<double, double>>> get_bs_location_container)
{
	bus_number = set_bus_number;									//車子的編號
	num_of_mt = set_num_of_mt;										//車內的總人數
	num_of_each_type_mt[0] = (num_of_mt / 10) * 2;
	num_of_each_type_mt[1] = (num_of_mt / 10) * 3;
	num_of_each_type_mt[2] = (num_of_mt / 10) * 3;
	num_of_each_type_mt[3] = (num_of_mt / 10) * 2;

	speed = set_speed;												//車子的速度: 36 km/hr = 0.1 km/10s = 0.0001 km/frame
	bs_location = get_bs_location_container;	

	moving_distance = speed /3600 * 10e-3;							//車子經過10 ms的移動距離: 0.0001 km	
	
/*--------------------------------------------------產生起始點,終點,及中間所有點-------------------------------------------------------*/
	path = 1.1;
	theta = unifrnd(0.0, 2 * PI);
// 	source = make_pair(0.0, 1.1);
// 	destination = make_pair(0.0, -1.1);

	source.first = path * cos(theta);			//generate initial point of a bus
	source.second = path * sin(theta);
	bus_location = source;
	destination.first = (0.0 - source.first);	//generate destination point of the bus
	destination.second = (0.0 - source.second);
	total_point_set.push_back(source);			//total_point_set現在有一個元素source
	
	//-------------------------------------------------------------------------
	alpha = -1;
	phi = -1;
	psi = -1;
	idx = 0;
	created_waypoint = make_pair(1000, 1000);
	center = make_pair(0.0, 0.0);

	while(total_point_set[idx] != destination)
	{
		do{
			
			alpha = unifrnd(0.2, 0.3);
			phi = unifrnd(-60, 60);
			
			if(destination.second - total_point_set[idx].second >= 0)
				psi = angle_between_vectors(make_pair((destination.first - total_point_set[idx].first), (destination.second - total_point_set[idx].second)), make_pair(1, 0));
			else
 				psi = 360 - angle_between_vectors(make_pair((destination.first - total_point_set[idx].first), (destination.second - total_point_set[idx].second)), make_pair(1, 0));							
			created_waypoint = make_pair(total_point_set[idx].first + alpha * cos((psi + phi) * PI / 180), total_point_set[idx].second + alpha * sin((psi + phi) * PI / 180));	

// 			cout  << idx << " " << "psi: " << psi << " phi:" << phi << "psi + phi:" << psi+phi << endl; 
// 			cout << "x" << alpha * cos((psi + phi) * PI / 180) << " " << "y " << alpha * sin((psi + phi) * PI / 180) << endl; 
		
		}while(distance_between_points(created_waypoint, center) >= 1.1);

		double d_temp = distance_between_points(total_point_set[idx], destination);

		if(d_temp > 0.3)
		{
			total_point_set.push_back(created_waypoint);
			total_angle_set.push_back( (psi + phi) * PI / 180  );						//注意!單位已換為徑度 
			temp_angle_set.push_back( (psi + phi - 180) * PI / 180   );					//注意!單位已換為徑度   
		}
		else
		{
			total_point_set.push_back(destination);
			total_angle_set.push_back(psi * PI / 180);									//注意!單位已換為徑度 
			temp_angle_set.push_back( (psi - 180) * PI / 180  );						//注意!單位已換為徑度  
		}

		idx++;
	}
	
	reverse_angle_set.resize(temp_angle_set.size(), 0);
	reverse_copy(temp_angle_set.begin(), temp_angle_set.end(), reverse_angle_set.begin());

	for(unsigned int n = 0; n < total_point_set.size(); n++)
		waiting_time_set.push_back(unifrnd(0.0, 0.5));			//sec
//**test the waiting time	
// 	for(unsigned int n = 0; n < total_point_set.size(); n++)
// 	{
// 		waiting_time_set.push_back(unifrnd(0.0, 0.5));
// 		cout << "wait time " << waiting_time_set[n] << endl;
// 	}
// 	system("PAUSE");

	reverse_waiting_time_set.resize(waiting_time_set.size(), 0);
	reverse_copy(waiting_time_set.begin(), waiting_time_set.end(), reverse_waiting_time_set.begin());

	reverse_point_set.resize(total_point_set.size(), make_pair(0,0));
	reverse_copy(total_point_set.begin(), total_point_set.end(), reverse_point_set.begin());

// 	reverse_order_of_tps.reserve(total_point_set.size());
// 	for(int m = total_point_set.size() - 1; m >= 0; m--)
// 		reverse_order_of_tps.push_back(total_point_set[m]);

/*----------------------test by yi-shing---------------------------------------*/
//	cout << "total_point_set.size " << total_point_set.size() << " " << "reverse_point_set.size " << reverse_point_set.size() << endl;
//	cout << "total_angle_set.size" << total_angle_set.size() << " " << "reverse_angle_set.size " << reverse_angle_set.size() << endl;
//	cout << total_point_set.back().first << " " << total_point_set.back().second << endl;
//	cout << reverse_point_set.back().first << " " << reverse_point_set.back().second << endl;
// 	cout << "total_angle_set:" << endl;
// 	for(unsigned int i = 0; i < total_angle_set.size(); i++)
// 		cout << total_angle_set[i] << endl;
// 	cout << endl << endl;
// 	for(unsigned int i = 0; i < temp_angle_set.size(); i++)
// 		cout << temp_angle_set[i] << endl;
//	cout << endl << endl;
// 	for(unsigned int i = 0; i < reverse_angle_set.size(); i++)
// 		cout << reverse_angle_set[i] << endl;
// 	system("PAUSE"); 

/*-----------------------------------------------------------------------------------------------------------------------------------*/
	/*test the correction of all points*/
// 	for(unsigned int i = 0; i < total_point_set.size(); i++)
// 		cout << total_point_set[i].first << " " << total_point_set[i].second << " " << endl;
//	for(int i = (int)reverse_order_of_tps.size()-1; i >= 0 ; i--)												//用unsigned會溢位出錯	
//		cout << reverse_order_of_tps[i].first << " " << reverse_order_of_tps[i].second << endl;
//	cout << system("PAUSE");

	/*------------------------------ mobility model用 ------------------------------*/
	ctw = total_point_set[1];										//初始化目前朝向的方向點
	ctw_number = 1;													//初始化目前朝向的方向點編號
	rtw = reverse_point_set[1];										//初始化回程時的朝向的方向點					
	rtw_number = 1;													//初始化回程時的朝向的方向點編號
	wtime_of_ctw = waiting_time_set[0];								//初始停留時間	
	wtime_number_of_ctw = 0;										//初始停留時間編號
	rwtime_of_rtw = reverse_waiting_time_set[0];
	rwtime_number_of_rtw = 0;
	tas_number = 0;													//初始角度編號
	ras_number = 0;													//初始回程角度編號
	initial_point = source;											//起點			
	end_point = destination;										//終點			
	//--------------------------------------------------------------------------------

	for(int j = 0;j < 3; j++)
	{
		flag[j] = false;				//判斷是否換手
		serving_bs_set[j] = -1;			//各個網路裡訊號最強的bs	
		old_serving_bs_set[j] = -1;
	}

	old_serving_network = -1;

	cumulate_num_of_handover = 0;

	int cumu_i = 0;
	for(int h = 0; h < 4; h++)
	{	
		for(int i = 0; i < num_of_each_type_mt[h]; i++)
		{
			mt_container.push_back(Mt(cumu_i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
			cumu_i++;
		}
	}
	
// 	for(int i = 0; i < num_of_mt; i++)
// 	{
// 		int h = rand()%4;				//h = 0, 1, 2, 3
// 		mt_container.push_back(Mt(i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
// //		alias_of_mt_container().push_back(Mt(i, h, make_pair(-1, -1), bus_number, bus_location, bs_location));
// 	}
}
Bus::~Bus(){}

/*--- 取用車子速度 ---*/
double Bus::get_speed()
{
	return speed;
}

/*--- 取用車子座標 ---*/
pair<double, double> Bus::get_bus_location()
{ 
	return bus_location; 
}

vector<vector<pair<double, double>>> Bus::get_bs_location()
{
	return bs_location;
}

/*--- 取用方向點 ---*/
pair<double, double> Bus::get_ctw()
{
	return ctw;
}

/*--- 取用方向點 ---*/
pair<double, double> Bus::get_rtw()
{
	return rtw;
}

pair<double, double> Bus::get_source()
{
	return source;
}

pair<double, double> Bus::get_destination()
{
	return destination;
}

int Bus::gctw_number()
{
	return ctw_number;
}

vector<pair<double, double>> &Bus::get_total_point_set()
{
	return total_point_set;
}

vector<double> Bus::get_waiting_time_set()
{
	return waiting_time_set;
}

bool *Bus::get_flag()
{
	return flag;
}

int Bus::get_cumulate_num_of_handover()
{
	return cumulate_num_of_handover;
}

int *Bus::get_serving_bs_set()
{
	return serving_bs_set;
}

int *Bus::get_old_serving_bs_set()
{
	return old_serving_bs_set;
}

int Bus::get_old_serving_network()
{
	return old_serving_network;
}

/*--- 取用mt數量 ---*/
int Bus::get_num_of_mt()
{
	return num_of_mt;
}

vector<Mt> &Bus::alias_of_mt_container()
{
	return mt_container;
}

/*--- 車子每單位frame的移動方式 (Modified Random WayPoint Mobility Model) ---*/
void Bus::mobility()
{
	if(initial_point == source)
	{
		if(waiting_time_set[wtime_number_of_ctw] > 0)						//bus still stays
			waiting_time_set[wtime_number_of_ctw] -= 10e-3;					//10e-3
		else
		{
			double d1 = distance_between_points(make_pair(bus_location.first, bus_location.second), make_pair(bus_location.first + moving_distance * cos(total_angle_set[tas_number]), bus_location.second + moving_distance * sin(total_angle_set[tas_number])));
			double d2 = distance_between_points(make_pair(bus_location.first, bus_location.second), make_pair(total_point_set[ctw_number].first, total_point_set[ctw_number].second));
			if( d1 < d2 )
			{
				/*--- 車子往方向點移動(改變車子的座標) ---*/
				bus_location.first += moving_distance * cos(total_angle_set[tas_number]);
				bus_location.second += moving_distance * sin(total_angle_set[tas_number]);

				/*--- 把最新的車子座標傳給mt知道 ---*/
				for(unsigned int i = 0; i < alias_of_mt_container().size(); i++)
					alias_of_mt_container()[i].get_car_location() = bus_location;
			}
			else
			{
				/*--- 車子移動到方向點(改變車子的座標) ---*/
				bus_location = total_point_set[ctw_number];

				/*--- 把最新的車子座標傳給mt知道 ---*/
				for(unsigned int i = 0; i < alias_of_mt_container().size(); i++)
					alias_of_mt_container()[i].get_car_location() = bus_location;

				if(bus_location != destination)
				{
					ctw_number++;
					tas_number++;
					wtime_number_of_ctw++;
					ctw = total_point_set[ctw_number];			//為了在main能印出來才用的參數,不然其實有ctw_number和total_point_set就夠了
				}
				else
				{
					initial_point = destination;
					end_point = source;
					tas_number = 0;
					wtime_number_of_ctw = 0;
					ctw_number = 1;
					ctw = total_point_set[ctw_number];			//為了在main能印出來
				}
			}
		}
	}
	else
	{		
//		system("PAUSE");
// 		if(reverse_waiting_time_set[rwtime_number_of_rtw] > 0)
// 			reverse_waiting_time_set[rwtime_number_of_rtw] -= 10e-3;
// 		else
// 		{
// 			double d3 = distance_between_points(make_pair(bus_location.first, bus_location.second), make_pair(bus_location.first + moving_distance * cos(reverse_angle_set[ras_number]),bus_location.second + moving_distance * sin(reverse_angle_set[ras_number])));
// 			double d4 = distance_between_points(make_pair(bus_location.first, bus_location.second), make_pair(reverse_point_set[rtw_number].first, reverse_point_set[rtw_number].second));
// 			if( d3 < d4)
// 			{
// 				/*--- 車子往方向點移動(改變車子的座標) ---*/
// 				bus_location.first += moving_distance * cos(reverse_angle_set[ras_number]);
// 				bus_location.second += moving_distance * sin(reverse_angle_set[ras_number]);
// 
// 				/*--- 把最新的車子座標傳給mt知道 ---*/
// 				for(unsigned int i = 0; i < alias_of_mt_container().size(); i++)
// 					alias_of_mt_container()[i].get_car_location() = bus_location;
// 			}
// 			else
// 			{
// 				/*--- 車子移動到方向點(改變車子的座標) ---*/
// 				bus_location = reverse_point_set[rtw_number];
// 
// 				/*--- 把最新的車子座標傳給mt知道 ---*/
// 				for(unsigned int i = 0; i < alias_of_mt_container().size(); i++)
// 					alias_of_mt_container()[i].get_car_location() = bus_location;
// 
// 				if(bus_location != destination)
// 				{
// 					rtw_number++;
// 					ras_number++;
// 					rwtime_number_of_rtw++;
// 					rtw = reverse_point_set[rtw_number];
// 				}
// 				else
// 				{
// 					initial_point = source;
// 					end_point = destination;
// 					ras_number = 0;
// 					rwtime_number_of_rtw = 0;
// 					rtw_number = 1;
// 					rtw = reverse_point_set[rtw_number];
// 				}
// 			}
// 		}		
	}
}


/*--- 車子每單位frame的移動方式 (Modified Random WayPoint Mobility Model) ---*/
// void Bus::mobility()
// {	
// 	
// 	if(initial_point == source && end_point == destination)
// 	{
// 		pair<double, double> old_bus_location = bus_location;
// 
// 		if(waiting_time_of_ctw > 0)							//bus still stays
// 			waiting_time_of_ctw -= 10e-3;					//10e-3
// 		else
// 		{	
// 			/*--- 計算新方向點和車子座標的夾角 (計算移動距離用) ---*/   
// 			angle = angle_between_vectors(make_pair(total_point_set[ctw_number].first - bus_location.first, total_point_set[ctw_number].second - bus_location.second), make_pair(1,0));
// 
// 			/*--- 車子往方向點移動(改變車子的座標) ---*/
// 			bus_location.first += moving_distance * cos(angle * PI / 180);
// 			bus_location.second += moving_distance * sin(angle * PI / 180);
// 
// 			/*--- 把最新的車子座標傳給mt知道 ---*/
// 			for(unsigned int i = 0; i < alias_of_mt_container().size(); i++)
// 				alias_of_mt_container()[i].get_car_location() = bus_location;
// 
// 			/*--- 若行駛超過方向點(此方向點不為終點)則改變方向點座標及夾角 ---*/
// 			if(distance_between_points(old_bus_location, total_point_set[ctw_number]) < distance_between_points(bus_location, old_bus_location)
// 				&& total_point_set[ctw_number] != destination)
// 			{
// 				/*--- 更新所到waypoint的等待時間 ---*/
// 				waiting_time_of_ctw = waiting_time_set[ctw_number]; 
// 
// 				/*--- 更新目前朝向的方向點 ---*/		
// 				ctw_number += 1;												//改變目前waypoint編號
// 				ctw.first = total_point_set[ctw_number].first;					//改變目前waypoint座標
// 				ctw.second = total_point_set[ctw_number].second;	
// 			}
// 			
// 			/*--- 若超出的方向點為目的點 ---*/
// 			else if(distance_between_points(old_bus_location, total_point_set[ctw_number]) < distance_between_points(bus_location, old_bus_location)
// 				&& total_point_set[ctw_number] == destination)
// 			{
// 				/*--- 互換出發地與目的地 ---*/
// 				temp_point = initial_point;
// 				initial_point = end_point;
// 				end_point = temp_point;
// 
// 				/*--- 改變等待時間 ---*/
// 				waiting_time_of_ctw = reverse_waiting_time_set[0];
// 				
// 				/*--- 改變方向點編號,座標 ---*/	
// 				ctw_number = 1;
// 				ctw.first = reverse_order_of_tps[ctw_number].first;				//改變目前waypoint座標
// 				ctw.second = reverse_order_of_tps[ctw_number].second;	
// 
// 			}
// 		}		
// 	}
// 	else 
// 	{
// 		pair<double, double> old_bus_location = bus_location;
// 
// 		if(waiting_time_of_ctw > 0)							//bus still stays
// 			waiting_time_of_ctw -= 10e-3;					//10e-3
// 		else
// 		{		
// 			/*--- 計算新方向點和車子座標的夾角 (計算移動距離用) ---*/   
// 			angle = angle_between_vectors(make_pair(reverse_order_of_tps[ctw_number].first - bus_location.first, reverse_order_of_tps[ctw_number].second - bus_location.second), make_pair(1,0));
// 
// 			/*--- 車子往方向點移動(改變車子的座標) ---*/
// 			bus_location.first += moving_distance * cos(angle * PI / 180);
// 			bus_location.second += moving_distance * sin(angle * PI / 180);
// 
// 			/*--- 把最新的車子座標傳給mt知道 ---*/
// 			for(unsigned int i = 0; i < alias_of_mt_container().size(); i++)
// 				alias_of_mt_container()[i].get_car_location() = bus_location;
// 
// 			/*--- 若行駛超過方向點則改變方向點座標及夾角 ---*/
// 			if(distance_between_points(old_bus_location, reverse_order_of_tps[ctw_number]) < distance_between_points(bus_location, old_bus_location)
// 				&& total_point_set[ctw_number] != destination)
// 			{
// 				/*--- 更新所到waypoint的等待時間 ---*/
// 				waiting_time_of_ctw = reverse_waiting_time_set[ctw_number]; 
// 
// 				/*--- 更新目前朝向的方向點 ---*/		
// 				ctw_number += 1;													//改變目前waypoint編號
// 				ctw.first = reverse_order_of_tps[ctw_number].first;					//改變目前waypoint座標
// 				ctw.second = reverse_order_of_tps[ctw_number].second;	
// 			}
// 			/*--- 若被超出的方向點為最後的目的地 ---*/
// 			else if(distance_between_points(old_bus_location, reverse_order_of_tps[ctw_number]) < distance_between_points(bus_location, old_bus_location)
// 				&& reverse_order_of_tps[ctw_number] == destination)
// 			{
// 				/*--- 互換出發地與目的地 ---*/
// 				temp_point = initial_point;
// 				initial_point = end_point;
// 				end_point = temp_point;
// 
// 				/*--- 改變等待時間 ---*/
// 				waiting_time_of_ctw = waiting_time_set[ctw_number];
// 
// 				/*--- 改變方向點編號,座標 ---*/	
// 				ctw_number = 1;
// 				ctw.first = total_point_set[ctw_number].first;				//改變目前waypoint座標
// 				ctw.second = total_point_set[ctw_number].second;	
// 			}
// 		}
// 	}
// }

/*--- 每單位frame移動完 判斷是否換手函數 ---*/
void Bus::ho_trigger()
{
	double tx_pilot_power[3] = {31, 33, 46};								//{gsm/edge, wcdma, lte}
	double rx_pilot_power[3] = {-1000, -1000, -1000};						//{gsm/edge, wcdma, lte}
	double d[3] = {-1, -1, -1};
	
	/*--- 計算車子跟serving_bs_set的距離,path loss(若小於最小距離則用最小距離),接收訊號強度 ---*/
	for(int j = 0; j < 3; j++)																	//{gsm/edge, wcdma, lte}
	{
		if(j == 0)																				//當serving network是 gsm 或 edge 
		{
			d[j] = distance_between_points(bs_location[j][serving_bs_set[j] - 1], bus_location);	
			if(d[j] < 0.02)
				d[j] = 0.02;
			rx_pilot_power[j] = tx_pilot_power[j] - gsm_edge_path_loss(d[j]);	

//			cout << "d[0]: " << d[j] << " tx: " << tx_pilot_power[j] << " gsm_edge_path_loss: " << gsm_edge_path_loss(d[j]) << " rx: " << rx_pilot_power[j] << endl;
		}
		else if(j == 1)																			//若serving network是 wcdma
		{
			d[j] = distance_between_points(bs_location[j][serving_bs_set[j] - 1], bus_location);
			if(d[j] < 0.015)
				d[j] = 0.015;
			rx_pilot_power[j] = tx_pilot_power[j] - wcdma_path_loss(d[j]);

//			cout << "d[1]: " << d[j] << " tx: " << tx_pilot_power[j] << "wcdma_path_loss: " << wcdma_path_loss(d[j]) << " rx: " << rx_pilot_power[j] << endl;
		}
		else if(j == 2)																			//若serving network是 lte					
		{
			d[j] = distance_between_points(bs_location[j][serving_bs_set[j] - 1], bus_location);
			if(d[j] < 0.012)
				d[j] = 0.012;
			rx_pilot_power[j] = tx_pilot_power[j] - lte_path_loss(d[j]);

//			cout << "d[2]: " << d[j]<< " tx: " << tx_pilot_power[j] << " lte_path_loss: " << lte_path_loss(d[j]) << " rx: " << rx_pilot_power[j] << endl;
		}
		/*--- test ---*/
//		cout << "bus_location: (" << bus_location.first << ", " << bus_location.second << ")" << endl;
// 		cout << "serving network: " << j << " rx_pilot_power: " << rx_pilot_power[j] << endl;
// 		system("PAUSE");

		/*--- 若serving bs接收訊號強度小於-90dBm則 1.flag變true  2.找出candidate network set ---*/
		if(rx_pilot_power[j] < -90)		  											
		{
			/*-------- 1. -------*/
			flag[j] = true;
			/*-------- 2. -------*/
// 			if(j = 0)
// 			{
// 				for(int i = 0; i < num_of_mt; i++)
// 				{
// 					if(mt_container[i].get_network_bs_number().first == 1 || mt_container[i].get_network_bs_number().first == 2)  
// 						cumulate_num_of_handover ++;
// 				}
// 			}
// 			else // j = 1, 2
// 			{
// 				for(int i = 0; i < num_of_mt; i++)
// 				{
// 					if(mt_container[i].get_network_bs_number().first == j + 2)  
// 						cumulate_num_of_handover ++;
// 				}
// 			}
		
			/*-------- 2. -------*/
			for(int jj = 0; jj < 3; jj++)
				old_serving_bs_set[jj] = serving_bs_set[jj];
			old_serving_network = j;

			double dis = -1;
			double pilot_power[3] = {-150, -150, -150};								//candidate network
			double largest_pilot_power[3] = {-120, -120, -120};
			int old_serving_bs[3] = {-1, -1, -1};
			old_serving_bs[j] = serving_bs_set[j] - 1;								//ex:1~27 -> 0~26
			
			for(unsigned int k = 0; k < bs_location[j].size(); k++)					//每個bs: 7, 19, 27個																	
			{
				if (k != old_serving_bs[j])											//不看原本serving network,BS	 	    
				{
					dis = distance_between_points(bus_location, bs_location[j][k]);
					pilot_power[j] = tx_pilot_power[j] - path_loss(j, dis);
					
					if (pilot_power[j] > largest_pilot_power[j])
					{
						largest_pilot_power[j] = pilot_power[j];
						serving_bs_set[j] = k + 1;									//更新成新的serving bs
					}
				}
			}
			/*-------------------*/
		}
	}
}

/*--- 兩點之間的距離 ---*/
double Bus::distance_between_points(const pair<double, double> &i, const pair<double, double> &j)
{
	return hypot(i.first - j.first, i.second - j.second);
}

/*--- 兩點之間的夾角 ---*/
double Bus::angle_between_points(const pair<double, double> &i, const pair<double, double> &j)		//j -> i
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

/*--- 兩向量之間的夾角 ---*/
double Bus::angle_between_vectors(const pair<double, double> &v1, const pair<double, double> &v2)
{
// 	double norm_v1 = hypot(v1.first, v1.second);
// 	double norm_v2 = hypot(v2.first, v2.second);
// 	double dot_product = (v1.first * v2.first) +(v1.second * v2.second);

	double norm_v1;
	double norm_v2;
	double dot_product;

	norm_v1 = hypot(v1.first, v1.second);
	norm_v2 = hypot(v2.first, v2.second);
	dot_product = (v1.first * v2.first) + (v1.second * v2.second);

	return acos(dot_product / (norm_v1 * norm_v2)) * 180 / PI;
}