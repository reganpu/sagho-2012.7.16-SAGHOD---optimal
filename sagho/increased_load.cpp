#include <iostream>
#include <cmath>
#include "increased_load.h"

using namespace std;

/*--- load intensity ---*/
double increased_load(int traffic_type, int network_type)
{
	double num_of_freq_channel = 18.0;	
	double num_of_t_channel = 10.0;
//	const double mean_data_rate[4] = {11.2e3 * (1.0/2.35), 64e3, 16.14e3, 88.9e3};	
	
	int total_num_of_freq_channel = 56;							
	int bit_per_slot = 114 * 3;					//342
	int slot_per_frame = 3;
	double max_transmission_rate = (bit_per_slot * slot_per_frame / 4.615e-3) * num_of_freq_channel;	//222.3 kbps * 18 = 4 Mbps

	const double Eb_N0[4] = {pow(10.0, 4.0/10), pow(10.0, 3.0/10), pow(10.0, 2.0/10), pow(10.0, 1.5/10)};
//	const double mean_data_rate[4] = {11.2e3 * (1.0/2.35), 64e3, 16.14e3, 88.9e3};
	const double chip_rate = 3.84e6;
	const double average_other_to_own_interference_ratio = 0.55;
	const double average_orthogonality_factor = 0.5;

	const double mean_data_rate[4] = {11.2e3 * (1.0/2.35), 64e3, 16.14e3, 88.9e3};
	const double required_RB[4] = {1.0, 1.0, 1.0, 2.0};
	const double total_number_of_RU = 150.0;
	const double bit_per_RU = 864.0; //12*12*6

	switch(network_type)
	{
	case 1:
		switch(traffic_type)
		{
		case 0:														//voice
			return 1.0 / (num_of_t_channel * num_of_freq_channel);	//180
			break;
		default:
			cout << "service type is wrong" << endl;
			break;
		}
		break;

	case 2:
		switch(traffic_type)
		{
		case 1:	//video
			return mean_data_rate[1] / max_transmission_rate;						
			break;
		case 2:	//http
			return mean_data_rate[2] / max_transmission_rate;						
			break;
		case 3:	//ftp
			return mean_data_rate[3] / max_transmission_rate;						
			break;
		}
		break;

	case 3:
		switch(traffic_type)
		{
		case 0:								
			return mean_data_rate[0] * (Eb_N0[0] / chip_rate) * ( (1 - average_orthogonality_factor) + average_other_to_own_interference_ratio);
			break;
		case 1:
			return mean_data_rate[1] * (Eb_N0[1] / chip_rate) * ( (1 - average_orthogonality_factor) + average_other_to_own_interference_ratio);
			break;
		case 2:
			return mean_data_rate[2] * (Eb_N0[2] / chip_rate) * ( (1 - average_orthogonality_factor) + average_other_to_own_interference_ratio);
			break;
		case 3:
			return mean_data_rate[3] * (Eb_N0[3] / chip_rate) * ( (1 - average_orthogonality_factor) + average_other_to_own_interference_ratio);
			break;
		}
		break;

	case 4:
		switch(traffic_type)
		{
		case 0:								
			return mean_data_rate[0] / (total_number_of_RU * bit_per_RU / 10e-3);
			break;
		case 1:
			return mean_data_rate[1] / (total_number_of_RU * bit_per_RU / 10e-3);
			break;
		case 2:
			return mean_data_rate[2] / (total_number_of_RU * bit_per_RU / 10e-3);
			break;
		case 3:
			return mean_data_rate[3] / (total_number_of_RU * bit_per_RU / 10e-3);
			break;
		}
		break;
	}

	return 0;
}


/*--- gsm load intensity ---*/
double g_increased_load(int traffic_type)
{
	int total_num_of_freq_channel = 56;			//GSM-1800每個營運商有56個頻道可用
	int num_of_freq_channel = 18;				//frequency reuse factor = 3
	int num_of_t_channel = 3 + 7;				//兩個Tx

	switch(traffic_type)
	{
	case 0:																//voice
		return 1.0 / (num_of_t_channel * num_of_freq_channel);	//1 / 180
		
		break;
	default:
		cout << "service type is wrong" << endl;
		break;
	}
	return 0;
}

/*--- edge load intensity ---*/
double e_increased_load(int traffic_type)
{
	const double mean_data_rate[3] = {64e3, 16.14e3, 88.9e3};	
	int total_num_of_freq_channel = 56;			
	int num_of_freq_channel = 18;				
	int bit_per_slot = 114 * 3;					//342
	int slot_per_frame = 3;
	double max_transmission_rate = (bit_per_slot * slot_per_frame / 4.615e-3) * num_of_freq_channel;	//222.3 kbps * 18 = 4 Mbps

	switch(traffic_type)
	{
	case 1:	//video
		return mean_data_rate[0] / max_transmission_rate;						
		break;
	case 2:	//http
		return mean_data_rate[1] / max_transmission_rate;						
		break;
	case 3:	//ftp
		return mean_data_rate[2] / max_transmission_rate;						
		break;
	}
	return 0;
}

/*--- wcdma load intensity ---*/
double w_increased_load(int traffic_type)
{
	const double Eb_N0[4] = {pow(10.0, 4.0/10), pow(10.0, 3.0/10), pow(10.0, 2.0/10), pow(10.0, 1.5/10)};
	const double mean_data_rate[4] = {11.2e3 * (1.0/2.35), 64e3, 16.14e3, 88.9e3};
	const double chip_rate = 3.84e6;
	const double average_other_to_own_interference_ratio = 0.55;
	const double average_orthogonality_factor = 0.5;

	switch(traffic_type)
	{
	case 0:								
		return mean_data_rate[0] * (Eb_N0[0] / chip_rate) * ( (1 - average_orthogonality_factor) + average_other_to_own_interference_ratio);
		break;
	case 1:
		return mean_data_rate[1] * (Eb_N0[1] / chip_rate) * ( (1 - average_orthogonality_factor) + average_other_to_own_interference_ratio);
		break;
	case 2:
		return mean_data_rate[2] * (Eb_N0[2] / chip_rate) * ( (1 - average_orthogonality_factor) + average_other_to_own_interference_ratio);
		break;
	case 3:
		return mean_data_rate[3] * (Eb_N0[3] / chip_rate) * ( (1 - average_orthogonality_factor) + average_other_to_own_interference_ratio);
		break;
	}
	return 0;
}

/*--- lte load intensity ---*/
double l_increased_load(int traffic_type)
{
	const double mean_data_rate[4] = {11.2e3 * (1.0/2.35), 64e3, 16.14e3, 88.9e3};
//	const double required_RB[4] = {1.0, 1.0, 1.0, 2.0};
	const double total_number_of_RU = 150.0;
	const double bit_per_RU = 864.0; //12*12*6

	switch(traffic_type)
	{
	case 0:								
		return mean_data_rate[0] / (total_number_of_RU * bit_per_RU / 10e-3);
		break;
	case 1:
		return mean_data_rate[1] / (total_number_of_RU * bit_per_RU / 10e-3);
		break;
	case 2:
		return mean_data_rate[2] / (total_number_of_RU * bit_per_RU / 10e-3);
		break;
	case 3:
		return mean_data_rate[3] / (total_number_of_RU * bit_per_RU / 10e-3);
		break;
	}
	return 0;
}