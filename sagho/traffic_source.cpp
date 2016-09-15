#include <iostream>
#include <cmath>
#include "distributions.h"
#include "traffic_source.h"

using namespace std;

//time unit: frame
//the last element of queue "mt_buffer" is the object which stores the arrival time of next coming packet

double truncated_pareto(const double, const double, const double);
double truncated_lognrnd(const double, const double, const double, const double);

double truncated_pareto(const double index, const double min, const double max)
{
	double x;
	x = paretornd(index, min);
	if (x > max)
		x = max;
	return x;
}
double truncated_lognrnd(const double mean, const double var, const double min, const double max)
{
	double x;    
	x = lognrnd(mean, var);
	if (x < min)
		x = min;
	else if (x > max)
		x = max;
	return x;
}

void voice(queue<Packet> &buffer, const double frame_size)				//frame_size = WCDMA_frame_size = 10 ms
{
	const int packet_size = 28 * 8;										//voice的每個packet大小為固定值: 28 bytes = 224 bits
	const double on_period_mean = 1, off_period_mean = 1.35;			//平均值:1秒; 1.35秒
	const double interval_packet = 20e-3;								//interval_packet: inter arrival time between packets during on period
	
	int num_of_on_frame, num_of_off_frame;
	int next_on_frame, next_packet_arrival_frame;

	do {
		num_of_on_frame = int(floor(exprnd(on_period_mean) / frame_size));		//產生on frame數 = (on time / frame size); //on有幾個frame
	} while (num_of_on_frame == 0);

	do {
		num_of_off_frame = int(floor(exprnd(off_period_mean) / frame_size));	//產生off frame數
	} while (num_of_off_frame == 0);

	//下一次on是在第幾個frame = 初始值0 + 產生的on frame + 產生的off frame
	next_on_frame = buffer.back().get_arrival_time() + num_of_on_frame + num_of_off_frame;				

	//在on period中不斷產生間隔兩個frame的packets
	do {																		
		buffer.back().get_size() = packet_size;									//把曾經push進buffer過的一個初始packet的size 0改成 224
		if ((num_of_on_frame -= int(floor(interval_packet / frame_size))) >= 0)	//on frame數 - 2個frame數 (on frame數 大於等於 2個frame數)
		{
			//下一次packet到達時間 = 初始到達時間0 + 2個frame數
			next_packet_arrival_frame = buffer.back().get_arrival_time() + int(floor(interval_packet / frame_size));	
			buffer.push(Packet(0, next_packet_arrival_frame));					//利用 下一次packet到達時間 產生一個空packet
		}
		else																	//(on frame數 小於 2個frame數)
		{
			buffer.push(Packet(0, next_on_frame));								//利用 下一次on是在第幾個frame 產生一個空packet
		}
	} while (num_of_on_frame >= 0);
}

void video(queue<Packet> &buffer, const double frame_size)
{
	int i;
	const int num_of_packet_per_frame = 8;
	const double interval_frame = 100e-3;
	const int size_min = 40, size_max = 250;
	const double size_index = 1.2;
	const double interval_min = 2.5e-3, interval_max = 12.5e-3, interval_index = 1.2;
	int packet_size;
	int next_frame_arrival; //unit: frame
	double interval_packet, next_packet_arrival; //unit sec.

	next_frame_arrival = buffer.back().get_arrival_time() + int(ceil(interval_frame / frame_size)); //frame time
	next_packet_arrival = buffer.back().get_arrival_time() * frame_size; //sec. for accumulating time
	
	for (i = 1; i <= num_of_packet_per_frame; i++)
	{
		packet_size = int(floor(truncated_pareto(size_index, size_min, size_max))); //generate packet
		buffer.back().get_size() = (packet_size * 8);
		
		//determine the arrival time of next packet
		if (i != num_of_packet_per_frame)
		{
			interval_packet = truncated_pareto(interval_index, interval_min, interval_max);
			next_packet_arrival += interval_packet;
			buffer.push(Packet(0, int(ceil(next_packet_arrival / frame_size))));
			//cout << int(ceil(next_packet_arrival / frame_size)) << endl;
		}
		else
		{
			buffer.push(Packet(0, next_frame_arrival));
		}
	}
}

void http(queue<Packet> &buffer, const double frame_size)
{
	int i;
	const int SM_mean = 10710, SM_std = 25032, SM_min = 100, SM_max = int(2e6); //main object size (byte), lognormal distribution
	const int SE_mean = 7758, SE_std = 126168, SE_min = 50, SE_max = int(2e6); //embedded object size (byte), lognormal distribution
	const int ND_min = 2, ND_max = 53; //number of embedded objects per page, pareto distribution
	const double ND_index = 1.1;
	const double reading_time = 30, parsing_time = 0.13; //mean
	const int MTU = 576, MTU_max = 1500, header = 0; //unit: byte
	const double prob_of_MTU = 0.24;
	int SM, SE, ND, payload;
	int next_object_arrival;

	//determine payload size
	if (binornd(prob_of_MTU) == 1) //1: MTU, 0: MTU_max
		payload = MTU - header;
	else
		payload = MTU_max - header;
	
	//generate the main object
	SM = int(floor(truncated_lognrnd(SM_mean, pow(double(SM_std),2), SM_min, SM_max)));
	
	//divide the main object into packets
	do{
		if (SM > payload)
		{
			buffer.back().get_size() = (payload + header) * 8;
			buffer.push(Packet(0, buffer.back().get_arrival_time()));
		}
		else
		{
			buffer.back().get_size() = (SM + header) * 8;
			next_object_arrival = buffer.back().get_arrival_time() + int(ceil(exprnd(parsing_time) / frame_size));
			buffer.push(Packet(0, next_object_arrival));
		}
	}while ((SM -= payload) > 0);

	//number of the embedded objects
	ND = int(floor(truncated_pareto(ND_index, ND_min, ND_max)));
	
	for (i = 1; i <= ND; i++)
	{
		//generate the embedded object        
		SE = int(floor(truncated_lognrnd(SE_mean, pow(double(SE_std),2), SE_min, SE_max)));
		
		//divide the embedded object into packets
		do{
			if (SE > payload)
			{
				buffer.back().get_size() = (payload + header) * 8;
				buffer.push(Packet(0, buffer.back().get_arrival_time()));
			}
			else
			{
				buffer.back().get_size() = (SE + header) * 8;
				if (i != ND)
					next_object_arrival = buffer.back().get_arrival_time() + int(ceil(exprnd(parsing_time) / frame_size));
				else
					next_object_arrival = buffer.back().get_arrival_time() + int(ceil(exprnd(reading_time) / frame_size)); //new page arrival time
				buffer.push(Packet(0, next_object_arrival));
			}
		}while ((SE -= payload) > 0);
	}
}

void ftp(queue<Packet> &buffer, const double frame_size)
{
	const int file_min = 50, file_max = int(5e6), file_mean = int(2e6), file_std = int(0.772e6); //file size (byte)
	const int MTU1 = 576, MTU2 = 1500, header = 0;						//MTU: Maximum Transmission Unit (byte):file切成packet的兩種格式
	//1500: 以太網Ethernet信息包最大值，一般的設備（系統）也是默認值。576: 撥號連接到ISP的標準值
	const double prob_of_MTU1 = 0.24, mean_reading_time = 180;			//180 seconds
	int file_size, payload;
	int next_file_arrival;

	//determine payload size(packet size)
	if (binornd(prob_of_MTU1) == 1) //1: MTU, 0: MTU2
		payload = MTU1 - header;
	else
		payload = MTU2 - header;
	
	//generate file
	file_size = int(floor(truncated_lognrnd(file_mean, pow(double(file_std),2), file_min, file_max)));
	
	//divide file into packets
	do {
		if (file_size > payload)
		{
			buffer.back().get_size() = (payload + header) * 8;
			buffer.push(Packet(0, buffer.back().get_arrival_time()));					
		}
		else
		{
			buffer.back().get_size() = (file_size + header) * 8;
			next_file_arrival = buffer.back().get_arrival_time() + int(ceil(exprnd(mean_reading_time) / frame_size));
			buffer.push(Packet(0, next_file_arrival));
		}
	} while ((file_size -= payload) > 0);
}