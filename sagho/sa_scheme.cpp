#include "sa_scheme.h"

extern double GSM_loading_upper_bound, EDGE_loading_upper_bound, WCDMA_loading_upper_bound, LTE_loading_upper_bound;
const double Tmin = 0.1;
const double gamma = 0.8;
const int L = 10;	//100		//debug用5看看
const int C = 10;

void sa_scheme(Bus &bus, vector<vector<int>> &list, int frame, double dwelling_time[],
			   double &gsm_load_intensity, double &edge_load_intensity, double &wcdma_load_intensity, double &lte_load_intensity, 
		 	   vector<vector<double> > &network_average_delay, vector<vector<double> > &network_avg_drop_rate, vector<vector<double> > &net_avg_trans_rate,
		 	   int &num_of_blocked_mt)
//bus:某台車 list: 某台車的traffic type, mt number 
{
	/*===========================================================================================================================*/
	cout << endl << "========================= SAGHOD-optimal scheme ==============================" << endl;
	cout << "Before GHO, load intensity: " << gsm_load_intensity << " " << edge_load_intensity << " " << wcdma_load_intensity << " " << lte_load_intensity << endl;
	/*===========================================================================================================================*/

	vector<double> load_intensity, after_swap_load_intensity, load_intensity_of_best_solution;
	load_intensity.assign(4, 0);
	load_intensity[0] = gsm_load_intensity;
	load_intensity[1] = edge_load_intensity;
	load_intensity[2] = wcdma_load_intensity;
	load_intensity[3] = lte_load_intensity;

	after_swap_load_intensity.assign(4, 0);
	load_intensity_of_best_solution.assign(4, 0);

	double dwelling_time_expand[4] = {dwelling_time[0], dwelling_time[0], dwelling_time[1], dwelling_time[2]};

	double upperbound[4] = {GSM_loading_upper_bound, EDGE_loading_upper_bound, WCDMA_loading_upper_bound, LTE_loading_upper_bound};
//	double upperbound[4] = {1.0, 0.9, 0.7, 0.9};
	//test
// 	cout << upperbound[0] << " " << upperbound[1] << " " << upperbound[2] << " " << upperbound[3] << endl;
// 	system("PAUSE");

//	double mean_residual_commu_life_time = 60000 * 10e-3;		//600 sec
	
	double delay_constraint[2] = {40e-3, 100e-3};				//sec
	double dropped_rate_constraint = 0.01;						//1%
	double min_trans_rate[2] = {56e3, 100e3};					//bps

	int total_num_of_group_mt = 0;

	int num_of_blocked_mt_of_ini_solution = 0;
	double gho_blocking_ratio_of_ini_solution = 0;
	double gho_blocking_factor_of_ini_solution = 0;
	double dwelling_time_factor_of_ini_solution = 0;
	double QoS_factor_of_ini_solution = 0;
	double cost_function_value_of_ini_solution = 0;
	
 	int num_of_blocked_mt_of_new_solution = 0;
	int num_of_blocked_mt_of_best_solution;
 	double gho_blocking_ratio_of_new_solution = 0;
 	double gho_blocking_factor_of_new_solution = 0;
	double dwelling_time_factor_of_new_solution = 0;
	double QoS_factor_of_new_solution = 0;
	double cost_function_value_of_new_solution = 0;

	double sum_of_dwelling_time_ratio_of_ini_solution = 0;
	double average_dwelling_time_ratio_of_ini_solution = 0;
	double variance_of_dwelling_time_ratio_of_ini_solution = 0;

	double average_dwelling_time_ratio_of_new_solution = 0;
	double sum_of_dwelling_time_ratio_of_new_solution = 0;
	double variance_of_dwelling_time_ratio_of_new_solution = 0;

	double average_load_intensity_of_new_solution = 0;
	double load_balancing_factor_of_new_solution = 0;
	double sum_of_QoS_factor_of_new_solution = 0;

	double average_load_intensity_of_ini_solution = 0;
	double load_balancing_factor_of_ini_solution = 0;
	double sum_of_QoS_factor_of_ini_solution = 0;

	//vohvif, mt, (net, bs)
	vector<vector<vector<pair<int, int> > > > vohvif_mt_joinable_net_bs_set(4);
	vector<vector<vector<pair<int, int> > > > vohvif_mt_selected_net_bs(4); 
	vector<vector<vector<pair<int, int> > > > solution_before_swap(4);
	vector<vector<vector<pair<int, int> > > > solution_after_swap(4);
	vector<vector<vector<pair<int, int> > > > best_solution(4);

	vector<vector<vector<double> > > load_balancing_factor(4);
	vector<vector<vector<double> > > dwelling_time_ratio(4);
	vector<vector<vector<double> > > dwelling_time_factor(4);
	vector<vector<vector<double> > > selected_dwelling_time_ratio(4);
	vector<vector<vector<double> > > QoS_factor(4);
	vector<vector<vector<double> > > selected_QoS_factor(4);
	vector<vector<vector<pair<double, pair<int, int> > > > > vohvif_mt_pref_func_net_bs_set(4);

	vector<vector<vector<double> > > after_swap_dwelling_time_ratio(4);
	vector<vector<vector<double> > > after_swap_QoS_factor(4);

	after_swap_dwelling_time_ratio[0].assign(list[0].size(), vector<double>(1));
	after_swap_dwelling_time_ratio[1].assign(list[1].size(), vector<double>(1));
	after_swap_dwelling_time_ratio[2].assign(list[2].size(), vector<double>(1));
	after_swap_dwelling_time_ratio[3].assign(list[3].size(), vector<double>(1));

	after_swap_QoS_factor[0].assign(list[0].size(), vector<double>(1));
	after_swap_QoS_factor[1].assign(list[1].size(), vector<double>(1));
	after_swap_QoS_factor[2].assign(list[2].size(), vector<double>(1));
	after_swap_QoS_factor[3].assign(list[3].size(), vector<double>(1));

	vohvif_mt_joinable_net_bs_set[0].assign(list[0].size(), vector<pair<int, int> >(0, make_pair(0, 0)));	//* (0, make_pair(0,0)) = 待push 空的就代表沒網路符合
	vohvif_mt_joinable_net_bs_set[1].assign(list[1].size(), vector<pair<int, int> >(0, make_pair(0, 0)));
	vohvif_mt_joinable_net_bs_set[2].assign(list[2].size(), vector<pair<int, int> >(0, make_pair(0, 0)));
	vohvif_mt_joinable_net_bs_set[3].assign(list[3].size(), vector<pair<int, int> >(0, make_pair(0, 0)));

	//list[h][i]存mt編號, joinable_set_vohvif_mt_net_bs[h][i][k]存此mt的joinable network set 

	vohvif_mt_selected_net_bs[0].assign(list[0].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	vohvif_mt_selected_net_bs[1].assign(list[1].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	vohvif_mt_selected_net_bs[2].assign(list[2].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	vohvif_mt_selected_net_bs[3].assign(list[3].size(), vector<pair<int, int> >(1, make_pair(0, 0)));

	solution_before_swap[0].assign(list[0].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	solution_before_swap[1].assign(list[1].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	solution_before_swap[2].assign(list[2].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	solution_before_swap[3].assign(list[3].size(), vector<pair<int, int> >(1, make_pair(0, 0)));

	solution_after_swap[0].assign(list[0].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	solution_after_swap[1].assign(list[1].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	solution_after_swap[2].assign(list[2].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	solution_after_swap[3].assign(list[3].size(), vector<pair<int, int> >(1, make_pair(0, 0)));

	best_solution[0].assign(list[0].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	best_solution[1].assign(list[1].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	best_solution[2].assign(list[2].size(), vector<pair<int, int> >(1, make_pair(0, 0)));
	best_solution[3].assign(list[3].size(), vector<pair<int, int> >(1, make_pair(0, 0)));

	vohvif_mt_pref_func_net_bs_set[0].assign(list[0].size(), vector<pair<double, pair<int, int> > >(0, make_pair(0, make_pair(0, 0))));
	vohvif_mt_pref_func_net_bs_set[1].assign(list[1].size(), vector<pair<double, pair<int, int> > >(0, make_pair(0, make_pair(0, 0))));
	vohvif_mt_pref_func_net_bs_set[2].assign(list[2].size(), vector<pair<double, pair<int, int> > >(0, make_pair(0, make_pair(0, 0))));
	vohvif_mt_pref_func_net_bs_set[3].assign(list[3].size(), vector<pair<double, pair<int, int> > >(0, make_pair(0, make_pair(0, 0))));

	load_balancing_factor[0].assign(list[0].size(), vector<double>(1));
	load_balancing_factor[1].assign(list[1].size(), vector<double>(1));
	load_balancing_factor[2].assign(list[2].size(), vector<double>(1));
	load_balancing_factor[3].assign(list[3].size(), vector<double>(1));

	dwelling_time_ratio[0].assign(list[0].size(), vector<double>(0));
	dwelling_time_ratio[1].assign(list[1].size(), vector<double>(0));
	dwelling_time_ratio[2].assign(list[2].size(), vector<double>(0));
	dwelling_time_ratio[3].assign(list[3].size(), vector<double>(0));

	dwelling_time_factor[0].assign(list[0].size(), vector<double>(0));
	dwelling_time_factor[1].assign(list[1].size(), vector<double>(0));
	dwelling_time_factor[2].assign(list[2].size(), vector<double>(0));
	dwelling_time_factor[3].assign(list[3].size(), vector<double>(0));
	
	selected_dwelling_time_ratio[0].assign(list[0].size(), vector<double>(1, 1));
	selected_dwelling_time_ratio[1].assign(list[1].size(), vector<double>(1, 1));
	selected_dwelling_time_ratio[2].assign(list[2].size(), vector<double>(1, 1));
	selected_dwelling_time_ratio[3].assign(list[3].size(), vector<double>(1, 1));

	QoS_factor[0].assign(list[0].size(), vector<double>(0));
	QoS_factor[1].assign(list[1].size(), vector<double>(0));
	QoS_factor[2].assign(list[2].size(), vector<double>(0));
	QoS_factor[3].assign(list[3].size(), vector<double>(0));

	selected_QoS_factor[0].assign(list[0].size(), vector<double>(1));
	selected_QoS_factor[1].assign(list[1].size(), vector<double>(1));
	selected_QoS_factor[2].assign(list[2].size(), vector<double>(1));
	selected_QoS_factor[3].assign(list[3].size(), vector<double>(1));
	//vohvif, mt
	vector<pair<int, int> > list_of_first_selected_mt;
	vector<pair<int, int> > list_of_mt_having_dif_net;
	vector<pair<int, int> > list_of_mt_in_mt1_joinable_net_set;
	vector<pair<int, int> > list_of_mt_include_mt1_net;
	vector<pair<int, int> > list_of_mt_both_included_in_joinable_net_set;
	vector<pair<int, int> > list_of_mt2;

	/*======================================= test:list正確接收 ==============================================*/
	for(int ohif = 0; ohif < 4; ohif++)
	{
		cout << "list[" << ohif << "] have " << list[ohif].size() << " MTs: " << endl;
		for(unsigned int i = 0; i < list[ohif].size(); i++)
		{
			cout << "No." << bus.alias_of_mt_container()[list[ohif][i]].get_mt_number();
			cout << " type " << bus.alias_of_mt_container()[list[ohif][i]].get_traffic_type();
			cout << " Net: " << bus.alias_of_mt_container()[list[ohif][i]].get_network_bs_number().first; 
	        cout << " bs: " << bus.alias_of_mt_container()[list[ohif][i]].get_network_bs_number().second << endl;
		}
		cout << endl;
	}
	/*=======================================================================================================*/
	

	/*===================== test:load_intensity正確接收 ============================*/
// 	for(unsigned int j = 0; j < load_intensity.size(); j++)
// 		cout << "load_intensity[" << j << "] " << load_intensity[j] << " " << endl;
// 	system("PAUSE");
	/*============================================================================*/


	/*======================= test:dwelling_time正確接收 =========================*/
// 	for(int j = 0; j < 3; j++)
// 		cout << "dwelling_time[" << j << "] " << dwelling_time[j] << endl;
// 	system("PAUSE");
	/*===========================================================================*/

	/*===================================================== test:  ===============================================================*/
// 	cout << "for voice mt: " << endl; 
// 	cout << "candidate gsm:  avg voice delay " << network_average_delay[0][0] << " drop rate " << network_avg_drop_rate[0][0] << " l + delta(l)" << load_intensity[0] + increased_load(0, 1) << endl;
// 	cout << "candidate wcdma: avg voice delay " << network_average_delay[2][0] << " drop rate " << network_avg_drop_rate[2][0] << " l + delta(l)" << load_intensity[2] + increased_load(0, 3) << endl;
// 	cout << "candidate lte: avg voice delay " << network_average_delay[3][0] << " drop rate " << network_avg_drop_rate[3][0] << " l + delta(l)" << load_intensity[3] + increased_load(0, 4) << endl;
// 	cout << "for HTTP mt: " << endl;
// 	cout << "candidate edge: avg http trans rate " << net_avg_trans_rate[0][0] << " l + delta(l) " << load_intensity[1] + increased_load(2, 2) << endl;
// 	cout << "candidate wcdma: avg http trans rate " << net_avg_trans_rate[1][0] << " l + delta(l) " << load_intensity[2] + increased_load(2, 3) << endl;
// 	cout << "candidate lte: avg trans rate " << net_avg_trans_rate[2][0] << " l + delta(l) " << load_intensity[3] + increased_load(2, 4) << endl;				
// 	cout << "for video mt: " << endl;
// 	cout << "candidate edge: avg video delay " << network_average_delay[1][0] << " drop rate " << network_avg_drop_rate[1][0] << "l + delta(l)" << load_intensity[1] + increased_load(1, 2) << endl;
// 	cout << "candidate wcdms: avg video delay " << network_average_delay[2][1] << " drop rate " << network_avg_drop_rate[2][1] << "l + delta(l)" << load_intensity[2] + increased_load(1, 3) << endl;
// 	cout << "candidate lte: ave video delay " << network_average_delay[3][1] << " drop rate " << network_avg_drop_rate[3][1] << "l + delta(l)" << load_intensity[3] + increased_load(1, 4) << endl;
// 	cout << "for FTP mt: " << endl;
// 	cout << "candidate edge avg ftp trans rate " << net_avg_trans_rate[0][1] << " l + delta(l) " << load_intensity[1] + increased_load(3, 2) << endl;
// 	cout << "candidate wcdma avg ftp trans rate " << net_avg_trans_rate[1][1] << " l + delta(l) " << load_intensity[2] + increased_load(3, 3) << endl;
// 	cout << "candidate lte avg ftp trans rate " << net_avg_trans_rate[2][1] << " l + delta(l) " << load_intensity[3] + increased_load(3, 4) << endl;
// 	system("PAUSE");
	/*============================================================================================================================*/


								/*********************************************/
								/*         SA part1: Initialization          */
								/*********************************************/


	/*--- 找出每個gho mt的joinable network set ---*/
	for(unsigned int vohvif = 0; vohvif < list.size(); vohvif++)
	{	
		for(unsigned int i = 0; i < list[vohvif].size(); i++)
		{
			/*--- 計算list總人數(for GHO blocking factor) ---*/
			total_num_of_group_mt++;

			if(vohvif == 0)			//vo
			{	
				//check whether GSM, WCDMA, LTE satisfies the joinable network constraint  
				if(network_average_delay[0][0] <= delay_constraint[0] &&				//voice average delay in GSM 
				   network_avg_drop_rate[0][0] <= dropped_rate_constraint &&
				   load_intensity[0] + increased_load(0, 1) <= GSM_loading_upper_bound) 	
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(1, bus.get_serving_bs_set()[0]));
				if(network_average_delay[2][0] <= delay_constraint[0] &&				//voice average delay in WCDMA 
				   network_avg_drop_rate[2][0] <= dropped_rate_constraint &&
				   load_intensity[2] + increased_load(0, 3) <= WCDMA_loading_upper_bound) 
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(3, bus.get_serving_bs_set()[1]));
				if(network_average_delay[3][0] <= delay_constraint[0] &&				//voice average delay in LTE 
				   network_avg_drop_rate[3][0] <= dropped_rate_constraint &&
				   load_intensity[3] + increased_load(0, 4) <= LTE_loading_upper_bound) 
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(4, bus.get_serving_bs_set()[2]));
			}
			else if(vohvif == 1)	//http
			{	
				//edge
				if(net_avg_trans_rate[0][0] >= min_trans_rate[0] && load_intensity[1] + increased_load(2, 2) <= EDGE_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(2, bus.get_serving_bs_set()[0]));
				//wcdma
				if(net_avg_trans_rate[1][0] >= min_trans_rate[0] && load_intensity[2] + increased_load(2, 3) <= WCDMA_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(3, bus.get_serving_bs_set()[1]));
				//lte
				if(net_avg_trans_rate[2][0] >= min_trans_rate[0] && load_intensity[3] + increased_load(2, 4) <= LTE_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(4, bus.get_serving_bs_set()[2]));
			}
			else if(vohvif == 2)	//video    [1][0] [2][1] [3][1]
			{	
				//edge
				if(network_average_delay[1][0] <= delay_constraint[1] && network_avg_drop_rate[1][0] <= dropped_rate_constraint &&
					load_intensity[1] + increased_load(1, 2) <= EDGE_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(2, bus.get_serving_bs_set()[0]));
				//wcdma
				if(network_average_delay[2][1] <= delay_constraint[1] && network_avg_drop_rate[2][1] <= dropped_rate_constraint &&
					load_intensity[2] + increased_load(1, 3) <= WCDMA_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(3, bus.get_serving_bs_set()[1])); 	
				//lte
				if(network_average_delay[3][1] <= delay_constraint[1] && network_avg_drop_rate[3][1] <= dropped_rate_constraint &&
					load_intensity[3] + increased_load(1, 4) <= LTE_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(4, bus.get_serving_bs_set()[2]));
			}
			else //	vohvif == 3		//ftp	[0][1]  [1][1]  [2][1]
			{	
				//edge
				if(net_avg_trans_rate[0][1] >= min_trans_rate[1] && load_intensity[1] + increased_load(3, 2) <= EDGE_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(2, bus.get_serving_bs_set()[0]));
				//wcdma
				if(net_avg_trans_rate[1][1] >= min_trans_rate[1] && load_intensity[2] + increased_load(3, 3) <= WCDMA_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(3, bus.get_serving_bs_set()[1]));
				//lte
				if(net_avg_trans_rate[2][1] >= min_trans_rate[1] && load_intensity[3] + increased_load(3, 4) <= LTE_loading_upper_bound)
					vohvif_mt_joinable_net_bs_set[vohvif][i].push_back(make_pair(4, bus.get_serving_bs_set()[2]));	
			}

			/*=============================================================================================================*/
			cout << vohvif << ", " << i << ", " << " j_net_size: " << vohvif_mt_joinable_net_bs_set[vohvif][i].size() << ": ";
			for(unsigned int j_tmp = 0;j_tmp < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); j_tmp++)
				cout << vohvif_mt_joinable_net_bs_set[vohvif][i][j_tmp].first << " ";
			cout << endl;
			/*=============================================================================================================*/
		}
	}

	/*================================ test: joinable network set of each gho mt ========================================*/
// 	for(unsigned int vohvif = 0; vohvif < list.size(); vohvif++)
// 	{	
// 		for(unsigned int i = 0; i < list[vohvif].size(); i++)
// 		{
// 			cout << "mt " << list[vohvif][i] <<  " |joinable network bs set| " << vohvif_mt_joinable_net_bs_set[vohvif][i].size() << endl;	 
// 			for(unsigned int k = 0 ; k < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); k++)	 
// 				cout << "joinable network bs set[" << vohvif << "][" << i << "][" << k << "] = (" << vohvif_mt_joinable_net_bs_set[vohvif][i][k].first << ", " << vohvif_mt_joinable_net_bs_set[vohvif][i][k].second << ") " << endl;  
// 			cout << endl; 
// 		}
// 	}
// 	system("PAUSE");
	/*====================================================================================================================*/

	/*select a MT with min mean data rate, 依序從network with min preference function開始選*/	
	for(unsigned int vohvif = 0; vohvif < list.size(); vohvif++)	//每種type	
	{	
		for(unsigned int i = 0; i < list[vohvif].size(); i++)		//每個mt
		{
			cout << "vohvif " << vohvif << " i " << i << endl;
			/*--- 判斷有無joinable network ---*/
			if(vohvif_mt_joinable_net_bs_set[vohvif][i].size() == 0)
			{
				/*--- only use initialization part: assign新網路, bs ---*/
//				bus.alias_of_mt_container()[list[vohvif][i]].get_network_bs_number() = make_pair(0, 0);
				/*--- only use initialization part: 更新被block的總人數 ---*/
				num_of_blocked_mt_of_ini_solution++;
			}
			/*--- 若有joinable network ---*/
			if(vohvif_mt_joinable_net_bs_set[vohvif][i].size() != 0)
			{
				/*--- 計算MT的每個joinable network的preference function value ---*/
				/*--- 若mt為voice traffic ---*/
				if(vohvif == 0)	
				{
					/*--- 則用此算每個joinable network的preference function ---*/
					for(unsigned int j_index = 0; j_index < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); j_index++)
					{	
						/*--- 若joinable network為GSM ---*/
						if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 1)
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[0] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[0] += increased_load(0, 1);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[0] -= increased_load(0, 1);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = (delay_constraint[0] - network_average_delay[0][0]) / delay_constraint[0];	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(1, k)));	
						}	
						/*--- 若joinable network為WCDMA ---*/
						else if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 3)
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[1] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[2] += increased_load(0, 3);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[2] -= increased_load(0, 3);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = (delay_constraint[0] - network_average_delay[2][0]) / delay_constraint[0];	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second; 
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(3, k)));						
						}
						/*--- 若joinable network為LTE ---*/
						else if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 4)	
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[2] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[3] += increased_load(0, 4);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[3] -= increased_load(0, 4);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = (delay_constraint[0] - network_average_delay[3][0]) / delay_constraint[0];	
							QoS_factor[vohvif][i].push_back( 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(4, k)));
						}
					}/*--- 得到一個voice mt的joinable network set的preference function ---*/
					
					/*--- sort ---*/
					sort(vohvif_mt_pref_func_net_bs_set[vohvif][i].begin(), vohvif_mt_pref_func_net_bs_set[vohvif][i].end(), sort_pair2);
					
					/*--- 從preference function值最小的網路開始找滿足小於load intensity上限就得到vohvif_mt_selected_net_bs ---*/
					for(unsigned int j_index = 0; j_index < vohvif_mt_pref_func_net_bs_set[vohvif][i].size(); j_index++)
					{
						int j = vohvif_mt_pref_func_net_bs_set[vohvif][i][j_index].second.first;
						/*---　若滿足loading constraint　---*/
						if(load_intensity[j - 1] + increased_load(0, j) < upperbound[j - 1])
						{
							/*---　暫存此voice mt的網路,bs　---*/
							vohvif_mt_selected_net_bs[vohvif][i][0] = vohvif_mt_pref_func_net_bs_set[vohvif][i][j_index].second;
							/*--- 先幫part2計算cost function需要用到的東西 ---*/
							selected_dwelling_time_ratio[vohvif][i][0] = dwelling_time_ratio[vohvif][i][j_index];
							selected_QoS_factor[vohvif][i][0] = QoS_factor[vohvif][i][j_index];
							/*--- 更新load intensity of the selected network ---*/
							load_intensity[j - 1] += increased_load(0, j);
							
							/*====================================================== observe ========================================================================*/
							cout << "(joinable net, pref): ";
							if(vohvif_mt_joinable_net_bs_set[vohvif][i].size() == 0)
								cout << "no joinable network ";
							else
							{
								for(unsigned int jj = 0; jj < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); jj++)
									cout << vohvif_mt_joinable_net_bs_set[vohvif][i][jj].first << ", " << vohvif_mt_pref_func_net_bs_set[vohvif][i][jj].first << " ";
							}	
							cout << endl;
							int h = bus.alias_of_mt_container()[list[vohvif][i]].get_traffic_type();
							cout << "enter net " << j << ", load_intensity[" << j - 1 << "] + increased_load(" << h << ", " << j << ") = " << load_intensity[j - 1] << endl;
							/*=======================================================================================================================================*/
							
							break;
						}
						/*---　若不滿足loading constraint　---*/
						else
						{
							/*--- 若還沒找到可接受的最差的網路 ---*/
							if(j_index != vohvif_mt_pref_func_net_bs_set[vohvif][i].size() - 1)
								;//迴圈不做事 
							/*--- 若已經找到可接受的最差的網路 ---*/
							else
							{
								/*--- 此mt被block,　block人數+1 ---*/
								vohvif_mt_selected_net_bs[vohvif][i][0] = make_pair(0, 0);
								num_of_blocked_mt_of_ini_solution++;

								/*============================*/
								cout << "is blocked " << endl;
								/*============================*/
							}
						}
					}
					//debug
					cout << "voice mt " << list[vohvif][i] << " done " << endl;
				}

				/*--- 若mt為http traffic ---*/
				else if(vohvif == 1)	
				{
					/*--- 則用此算每個joinable network的preference function ---*/
					for(unsigned int j_index = 0; j_index < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); j_index++)
					{	
						/*--- 若joinable network為EDGE ---*/
						if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 2)	
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[0] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[1] += increased_load(2, 2);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[1] -= increased_load(2, 2);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = min((net_avg_trans_rate[0][0] / (10.0 * min_trans_rate[0])), 1.0);
							//QoS_factor[j] = exp(-QoS_degree);	
							QoS_factor[vohvif][i].push_back( 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(2, k)));							
						}	
						/*--- 若joinable network為WCDMA ---*/
						else if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 3)	
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[1] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[2] += increased_load(2, 3);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[2] -= increased_load(2, 3);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = min((net_avg_trans_rate[1][0] / (10.0 * min_trans_rate[0])), 1.0);
							//QoS_factor[j] = exp(-QoS_degree);	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(3, k)));			
						}
						/*--- 若joinable network為LTE ---*/
						else if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 4)	
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[2] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[3] += increased_load(2, 4);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[3] -= increased_load(2, 4);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = min((net_avg_trans_rate[2][0] / (10.0 * min_trans_rate[0])), 1.0);
							//QoS_factor[j] = exp(-QoS_degree);	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(4, k)));
						}
					}/*--- 得到一個HTTP mt的joinable network set的preference function ---*/

					/*--- sort ---*/
					sort(vohvif_mt_pref_func_net_bs_set[vohvif][i].begin(), vohvif_mt_pref_func_net_bs_set[vohvif][i].end(), sort_pair2);
					
					/*--- 從preference function值最小的網路開始找滿足小於load intensity上限就得到vohvif_mt_selected_net_bs ---*/
					for(unsigned int j_index = 0; j_index < vohvif_mt_pref_func_net_bs_set[vohvif][i].size(); j_index++)
					{
						int j = vohvif_mt_pref_func_net_bs_set[vohvif][i][j_index].second.first;
						
						/*---　若滿足loading constraint　---*/
						if(load_intensity[j - 1] + increased_load(2, j) < upperbound[j - 1])
						{
							/*---　暫存此HTTP mt的網路,bs　---*/
							vohvif_mt_selected_net_bs[vohvif][i][0] = vohvif_mt_pref_func_net_bs_set[vohvif][i][j_index].second;
							/*--- 先幫part2計算cost function需要用到的東西 ---*/
							selected_dwelling_time_ratio[vohvif][i][0] = dwelling_time_ratio[vohvif][i][j_index];
							selected_QoS_factor[vohvif][i][0] = QoS_factor[vohvif][i][j_index];
							/*--- step 3. update load intensity of the selected network ---*/
							load_intensity[j - 1] += increased_load(2, j);
							
							/*====================================================== observe ========================================================================*/
							cout << "(joinable net, pref): ";
							if(vohvif_mt_joinable_net_bs_set[vohvif][i].size() == 0)
								cout << "no joinable network ";
							else
							{
								for(unsigned int jj = 0; jj < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); jj++)
									cout << vohvif_mt_joinable_net_bs_set[vohvif][i][jj].first << ", " << vohvif_mt_pref_func_net_bs_set[vohvif][i][jj].first << " ";
							}	
							cout << endl;
							int h = bus.alias_of_mt_container()[list[vohvif][i]].get_traffic_type();
							cout << "enter net " << j << ", load_intensity[" << j - 1 << "] + increased_load(" << h << ", " << j << ") = " << load_intensity[j - 1] << endl;
							/*=======================================================================================================================================*/
							
							break;
						}
						else
						{
							/*--- 若還沒找到可接受的最差的網路 ---*/
							if(j_index != vohvif_mt_pref_func_net_bs_set[vohvif][i].size() - 1)
								;//迴圈不做事 
							/*--- 若已經找到可接受的最差的網路 ---*/
							else
							{
								/*--- 此mt被block,　block人數+1 ---*/
								vohvif_mt_selected_net_bs[vohvif][i][0] = make_pair(0, 0);
								num_of_blocked_mt_of_ini_solution++;

								/*============================*/
								cout << "is blocked " << endl;
								/*============================*/
							}
						}
					}
					//debug
					cout << "HTTP mt " << list[vohvif][i] << " done " << endl;
				}

				/*--- 若mt為video traffic ---*/
				else if(vohvif == 2)	//	  [1][0] [2][1] [3][1]
				{
					/*--- 則用此算每個joinable network的preference function ---*/
					for(unsigned int j_index = 0; j_index < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); j_index++)
					{	
						/*--- 若joinable network為EDGE ---*/
						if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 2)
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[0] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[1] += increased_load(1, 2);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[1] -= increased_load(1, 2);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = (delay_constraint[1] - network_average_delay[1][0]) / delay_constraint[1];	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(2, k)));	
						}	

						/*--- 若joinable network為WCDMA ---*/
						else if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 3)	
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[1] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[2] += increased_load(1, 3);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[2] -= increased_load(1, 3);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = (delay_constraint[1] - network_average_delay[2][1]) / delay_constraint[1];	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(3, k)));						
						}

						/*--- 若joinable network為LTE ---*/
						else if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 4)	
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[2] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[3] += increased_load(1, 4);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[3] -= increased_load(1, 4);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = (delay_constraint[1] - network_average_delay[3][1]) / delay_constraint[1];	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(4, k)));			
						}
					}/*--- 得到一個video mt的joinable network set的preference function ---*/

					/*--- sort ---*/
					sort(vohvif_mt_pref_func_net_bs_set[vohvif][i].begin(), vohvif_mt_pref_func_net_bs_set[vohvif][i].end(), sort_pair2);
					
					/*--- 從preference function值最小的網路開始找滿足小於load intensity上限就得到vohvif_mt_selected_net_bs ---*/
					for(unsigned int j_index = 0; j_index < vohvif_mt_pref_func_net_bs_set[vohvif][i].size(); j_index++)
					{
						int j = vohvif_mt_pref_func_net_bs_set[vohvif][i][j_index].second.first;
						
						/*---　若滿足loading constraint　---*/
						if(load_intensity[j - 1] + increased_load(1, j) < upperbound[j - 1])
						{
							/*---　暫存此HTTP mt的網路,bs　---*/
							vohvif_mt_selected_net_bs[vohvif][i][0] = vohvif_mt_pref_func_net_bs_set[vohvif][i][j_index].second;
							
							/*--- 先幫part2計算cost function需要用到的東西 ---*/
							selected_dwelling_time_ratio[vohvif][i][0] = dwelling_time_ratio[vohvif][i][j_index];
							selected_QoS_factor[vohvif][i][0] = QoS_factor[vohvif][i][j_index];
							
							/*--- step 3. update load intensity of the selected network ---*/
							load_intensity[j - 1] += increased_load(1, j);
							
							/*====================================================== observe ========================================================================*/
							cout << "(joinable net, pref): ";
							if(vohvif_mt_joinable_net_bs_set[vohvif][i].size() == 0)
								cout << "no joinable network ";
							else
							{
								for(unsigned int jj = 0; jj < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); jj++)
									cout << vohvif_mt_joinable_net_bs_set[vohvif][i][jj].first << ", " << vohvif_mt_pref_func_net_bs_set[vohvif][i][jj].first << " ";
							}	
							cout << endl;
							int h = bus.alias_of_mt_container()[list[vohvif][i]].get_traffic_type();
							cout << "enter net " << j << ", load_intensity[" << j - 1 << "] + increased_load(" << h << ", " << j << ") = " << load_intensity[j - 1] << endl;
							/*=======================================================================================================================================*/
							
							break;
						}
						/*---　若不滿足loading constraint　---*/
						else
						{
							/*--- 若還沒找到可接受的最差的網路 ---*/
							if(j_index != vohvif_mt_pref_func_net_bs_set[vohvif][i].size() - 1)
								;//迴圈不做事 
							/*--- 若已經找到可接受的最差的網路 ---*/
							else
							{
								/*--- 此mt被block,　block人數+1 ---*/
								vohvif_mt_selected_net_bs[vohvif][i][0] = make_pair(0, 0);
								num_of_blocked_mt_of_ini_solution++;

								/*============================*/
								cout << "is blocked " << endl;
								/*============================*/
							}
						}
					}
					//debug
					cout << "video mt " << list[vohvif][i] << " done " << endl;
				}
				/*--- 若mt為FTP traffic ---*/
				else //	vohvif == 3		//ftp	[0][1]  [1][1]  [2][1]
				{
					/*--- 則用此算每個joinable network的preference function ---*/
					for(unsigned int j_index = 0; j_index < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); j_index++)
					{	
						/*--- 若joinable network為EDGE ---*/
						if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 2)
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[0] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[1] += increased_load(3, 2);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[1] -= increased_load(3, 2);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = min((net_avg_trans_rate[0][1] / (10.0 * min_trans_rate[1])), 1.0);
							//QoS_factor[j] = exp(-QoS_degree);	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(2, k)));	
						}	

						/*--- 若joinable network為WCDMA ---*/
						else if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 3)	
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[1] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[2] += increased_load(3, 3);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[2] -= increased_load(3, 3);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = min((net_avg_trans_rate[1][1] / (10.0 * min_trans_rate[1])), 1.0);
							//QoS_factor[j] = exp(-QoS_degree);	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(3, k)));	
						}

						/*--- 若joinable network為LTE ---*/
						else if(vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].first == 4)	
						{
							/*--- single mt dwelling time factor ---*/
							dwelling_time_ratio[vohvif][i].push_back(min(dwelling_time[2] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0));
							dwelling_time_factor[vohvif][i].push_back(exp(- dwelling_time_ratio[vohvif][i].back()));
							/*--- single mt load balancing factor ---*/
							load_intensity[3] += increased_load(3, 4);
							double average_load = (load_intensity[0] + load_intensity[1] + load_intensity[2] + load_intensity[3] ) / 4;
							load_balancing_factor[vohvif][i][0] = sqrt((pow((load_intensity[0] - average_load), 2) + pow((load_intensity[1] - average_load), 2) + pow((load_intensity[2] - average_load), 2) + pow((load_intensity[3] - average_load), 2)) / 4);
							load_intensity[3] -= increased_load(3, 4);	
							/*--- single mt QoS factor ---*/
							double QoS_degree = min((net_avg_trans_rate[2][1] / (10.0 * min_trans_rate[1])), 1.0);
							//QoS_factor[j] = exp(-QoS_degree);	
							QoS_factor[vohvif][i].push_back(1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree))));

							double sum = dwelling_time_factor[vohvif][i].back() + load_balancing_factor[vohvif][i][0] + QoS_factor[vohvif][i].back() ;
							int k = vohvif_mt_joinable_net_bs_set[vohvif][i][j_index].second;
							vohvif_mt_pref_func_net_bs_set[vohvif][i].push_back(make_pair(sum, make_pair(4, k)));	
						}
					}/*--- 得到一個FTP mt的joinable network set的preference function ---*/

					/*--- sort ---*/
					sort(vohvif_mt_pref_func_net_bs_set[vohvif][i].begin(), vohvif_mt_pref_func_net_bs_set[vohvif][i].end(), sort_pair2);
					
					/*--- 從preference function值最小的網路開始找滿足小於load intensity上限就得到vohvif_mt_selected_net_bs ---*/
					for(unsigned int j_index = 0; j_index < vohvif_mt_pref_func_net_bs_set[vohvif][i].size(); j_index++)
					{
						int j = vohvif_mt_pref_func_net_bs_set[vohvif][i][j_index].second.first;
						
						/*---　若滿足loading constraint　---*/
						if(load_intensity[j - 1] + increased_load(3, j) < upperbound[j - 1])
						{
							vohvif_mt_selected_net_bs[vohvif][i][0] = vohvif_mt_pref_func_net_bs_set[vohvif][i][j_index].second;
							/*--- 先幫part2計算cost function需要用到的東西 ---*/
							selected_dwelling_time_ratio[vohvif][i][0] = dwelling_time_ratio[vohvif][i][j_index];
							selected_QoS_factor[vohvif][i][0] = QoS_factor[vohvif][i][j_index];
							/*--- 更新load intensity of the selected network ---*/
							load_intensity[j - 1] += increased_load(3, j);
											
							/*====================================================== observe ========================================================================*/
							cout << "(joinable net, pref): ";
							if(vohvif_mt_joinable_net_bs_set[vohvif][i].size() == 0)
								cout << "no joinable network ";
							else
							{
								for(unsigned int jj = 0; jj < vohvif_mt_joinable_net_bs_set[vohvif][i].size(); jj++)
									cout << vohvif_mt_joinable_net_bs_set[vohvif][i][jj].first << ", " << vohvif_mt_pref_func_net_bs_set[vohvif][i][jj].first << " ";
							}	
							cout << endl;
							int h = bus.alias_of_mt_container()[list[vohvif][i]].get_traffic_type();
							cout << "enter net " << j << ", load_intensity[" << j - 1 << "] + increased_load(" << h << ", " << j << ") = " << load_intensity[j - 1] << endl;
							/*=======================================================================================================================================*/
							
							break;
						}
						/*---　若不滿足loading constraint　---*/
						else
						{
							/*--- 若還沒找到可接受的最差的網路 ---*/
							if(j_index != vohvif_mt_pref_func_net_bs_set[vohvif][i].size() - 1)
								;//迴圈不做事 
							/*--- 若已經找到可接受的最差的網路 ---*/
							else
							{
								/*--- 此mt被block,　block人數+1 ---*/
								vohvif_mt_selected_net_bs[vohvif][i][0] = make_pair(0, 0);
								num_of_blocked_mt_of_ini_solution++;

								/*============================*/
								cout << "is blocked " << endl;
								/*============================*/

							}
						}
					}
					//debug
					cout << "FTP mt " << list[vohvif][i] << " done " << endl;
				}
			}
			/*--- 先幫part2計算cost function需要用到的東西 ---*/
			sum_of_dwelling_time_ratio_of_ini_solution += selected_dwelling_time_ratio[vohvif][i][0];
			sum_of_QoS_factor_of_ini_solution += selected_QoS_factor[vohvif][i][0];
		}
	}

	/*====================================================== debug =============================================================*/
	cout << "After ini, load intensity: " << load_intensity[0] << ", " << load_intensity[1] << ", " << load_intensity[2] << ", " << load_intensity[3] << endl;
	cout << "num_of_block_mt_of_ini_solution: " << num_of_blocked_mt_of_ini_solution << endl << endl;
//	system("PAUSE");
	/*==========================================================================================================================*/

	
										/*********************************************/
										/*    SA part2: generation and acceptance    */
										/*********************************************/


	/*--- 初始化最佳解的網路選擇,網路loading,block人數 ---*/
best_solution = vohvif_mt_selected_net_bs;
	load_intensity_of_best_solution = load_intensity;
	num_of_blocked_mt_of_best_solution = num_of_blocked_mt_of_ini_solution;

	/*--- initial解的cost function ---*/
	
	//group handover blocking factor
	gho_blocking_ratio_of_ini_solution = (double)num_of_blocked_mt_of_ini_solution / total_num_of_group_mt;
	if(gho_blocking_ratio_of_ini_solution > 0.05)
		gho_blocking_factor_of_ini_solution = ((gho_blocking_ratio_of_ini_solution - 0.05) / (1 - 0.05)) + 3;
	else
		gho_blocking_factor_of_ini_solution = gho_blocking_ratio_of_ini_solution;

	//dwelling time factor
	average_dwelling_time_ratio_of_ini_solution = sum_of_dwelling_time_ratio_of_ini_solution / (total_num_of_group_mt - num_of_blocked_mt_of_ini_solution);
	double temp_sum = 0;
	for(unsigned int vohvif = 0; vohvif < list.size(); vohvif++)
	{	
		for(unsigned int i = 0; i < list[vohvif].size(); i++)
		{
			if(vohvif_mt_joinable_net_bs_set[vohvif][i].size() != 0)		//******************************wrong????!!
				temp_sum += pow(selected_dwelling_time_ratio[vohvif][i][0] - average_dwelling_time_ratio_of_ini_solution, 2);
		}
	}
	variance_of_dwelling_time_ratio_of_ini_solution = sqrt(temp_sum / (total_num_of_group_mt - num_of_blocked_mt_of_ini_solution)); 
	dwelling_time_factor_of_ini_solution = exp(-average_dwelling_time_ratio_of_ini_solution + variance_of_dwelling_time_ratio_of_ini_solution);
	
	//load balancing factor
	double temp_load_sum = 0, temp_mean_square_sum = 0;
	for(int h = 0; h < 4; h++)
		temp_load_sum += load_intensity[h];
	average_load_intensity_of_ini_solution = temp_load_sum / 4;
	for(int h = 0; h < 4; h++)
		temp_mean_square_sum += pow(load_intensity[h] - average_load_intensity_of_ini_solution, 2);
	load_balancing_factor_of_ini_solution = sqrt(temp_mean_square_sum / 4);
	
	//QoS factor
	QoS_factor_of_ini_solution = sum_of_QoS_factor_of_ini_solution / (total_num_of_group_mt - num_of_blocked_mt_of_ini_solution);
	
	//cost function
	cost_function_value_of_ini_solution = gho_blocking_factor_of_ini_solution + dwelling_time_factor_of_ini_solution + load_balancing_factor_of_ini_solution + QoS_factor_of_ini_solution;
	
	/*======================================= test ===============================================*/
	cout << "*gho_blocking_factor_of_ini_solution: " << gho_blocking_factor_of_ini_solution << endl;
	cout << "*dwelling_time_factor_of_ini_solution: " << dwelling_time_factor_of_ini_solution << endl;
	cout << "*load_balancing_factor_of_ini_solution: " << load_balancing_factor_of_ini_solution << endl;
	cout << "*QoS_factor_of_ini_solution: " << QoS_factor_of_ini_solution << endl;
 	cout << "*cost_function_value_of_ini_solution: " << cost_function_value_of_ini_solution << endl;
	cout << "----------------------------------------------------" << endl;
//	system("PAUSE");
	/*============================================================================================*/

	double T = 3;	//5 debug用3看看
	
	solution_before_swap = vohvif_mt_selected_net_bs;   //用好懂的名字替代
 	solution_after_swap = vohvif_mt_selected_net_bs;	//在後面中間會被交換才是真正的交換後的解

	double T_best = -1;
	int l_best = -1;
	double gho_blocking_factor_best = gho_blocking_factor_of_ini_solution;
	double dwelling_time_factor_best = dwelling_time_factor_of_ini_solution;
	double load_balancing_factor_best = load_balancing_factor_of_ini_solution;
	double QoS_factor_best = QoS_factor_of_ini_solution;
	double cost_function_best = cost_function_value_of_ini_solution;


	while(T > Tmin)
	{
//		cout << "T " << T << endl;
		int c = 0;

		for(int l = 0; l < L; l++)
		{
//			cout << "l " << l << endl;

			list_of_first_selected_mt.clear();
			list_of_mt_having_dif_net.clear();
			list_of_mt_in_mt1_joinable_net_set.clear();
			list_of_mt_include_mt1_net.clear();
			list_of_mt_both_included_in_joinable_net_set.clear();
			list_of_mt2.clear();

			/*--- 把二維list變一維list ---*/
			for(unsigned int ohif = 0; ohif < list.size(); ohif ++)
			{
				for(unsigned int i = 0; i < list[ohif].size(); i++)
				{
					//push list的二維index進入一維pair
					list_of_first_selected_mt.push_back(make_pair(ohif, i));
					//test
//					cout << "list_of_first_selected_mt_set " << list_of_first_selected_mt.back().first << " " << list_of_first_selected_mt.back().second  
//						 << " network " << solution_before_swap[ohif][i][0].first << " bs " << solution_before_swap[ohif][i][0].second << endl;
				}
			}

			/*--- 再一維list中隨機選一個mt ---*/
			int list_index1 = unidrnd(0, list_of_first_selected_mt.size() - 1);
			
			/*--- save information of random selected MT: mt1 ---*/ 
			int vohvif1 = list_of_first_selected_mt[list_index1].first;
			int h1;
			if(vohvif1 == 0 || vohvif1 == 3)
				h1 = vohvif1;
			else if(vohvif1 == 1)
				h1 = 2;
			else	//vohvif1 == 2
				h1 = 1;
			int i1 = list_of_first_selected_mt[list_index1].second;
			int net1 = solution_before_swap[vohvif1][i1][0].first;

			/*================================================== test ===============================================================*/
//  			if(net1 != 0)
// 			{
// 				cout << "*1st selected mt: " << " h " << h1 << ",i " << i1 << ",net " << net1 
// 					 << ", load_int[" << net1 -1 << "] " << load_intensity[net1 -1] 
// 				     << "- incre load(" << h1 << ", " << net1 << ") " << increased_load(h1, net1) 
// 					 << " = " << load_intensity[net1 -1] - increased_load(h1, net1)  << endl;
// 			}
// 			else
// 				cout << "*1st selected mt: " << " h " << h1 << ",i " << i1 << ",net " << net1 << endl;
			/*=======================================================================================================================*/
			
			int list_index2;
			int vohvif2;
			int h2;
			int i2;
			int net2;

			/*--- 先找出網路不同於mt1的所有mt ---*/
			for(unsigned int ohif = 0; ohif < list.size(); ohif ++)
			{
				for(unsigned int i = 0; i < list[ohif].size(); i++)
				{
					//若其他mt選擇的網路不與mt1相同
					if(solution_before_swap[ohif][i][0].first != net1)
					{
						list_of_mt_having_dif_net.push_back(make_pair(ohif, i));
						
						//test
//						cout << "list_of_mt_having_dif_net " << list_of_mt_having_dif_net.back().first << " " << list_of_mt_having_dif_net.back().second 
//						     << " network " << solution_before_swap[ohif][i][0].first << " bs " << solution_before_swap[ohif][i][0].second << endl;
					}
				}
			}
			
			/*--- 判斷: 如果選到的mt1是被block的 ---*/
			if(net1 == 0)	
			{
				for(unsigned int onedimind = 0; onedimind < list_of_mt_having_dif_net.size(); onedimind++)
				{
					unsigned int ohif_temp = list_of_mt_having_dif_net[onedimind].first;
					unsigned int i_temp = list_of_mt_having_dif_net[onedimind].second;
					pair<int, int> temp_net_bs_2 = solution_before_swap[ohif_temp][i_temp][0]; 
					bool k = false;
					/*--- check mt2的網路是否包含在mt1 joinable network set裡面 (有多判斷一次一定不相同的) ---*/
					for(unsigned int mt1_net_index = 0; mt1_net_index < vohvif_mt_joinable_net_bs_set[vohvif1][i1].size(); mt1_net_index ++)
					{
						k = compare(temp_net_bs_2, vohvif_mt_joinable_net_bs_set[vohvif1][i1][mt1_net_index]);
						if(k == true)//代表網路相同 
						{
							list_of_mt_in_mt1_joinable_net_set.push_back(list_of_mt_having_dif_net[onedimind]);
							break;
						}
					}
				}
				//接著在有包含於joinable network set裡面的mt2,check mt2的load intensity是否可容納
				for(unsigned int onedimind = 0; onedimind < list_of_mt_in_mt1_joinable_net_set.size(); onedimind++)
				{
					int ohif_temp = list_of_mt_in_mt1_joinable_net_set[onedimind].first;  
					int i_temp = list_of_mt_in_mt1_joinable_net_set[onedimind].second;
					int net_temp = solution_before_swap[ohif_temp][i_temp][0].first;
					if(ohif_temp == 0)	//voice
					{
						if(load_intensity[net_temp - 1] - increased_load(0, net_temp) + increased_load(h1, net_temp) <= upperbound[net_temp - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
					else if(ohif_temp == 1)	//http 
					{
						if(load_intensity[net_temp - 1] - increased_load(2, net_temp) + increased_load(h1, net_temp) <= upperbound[net_temp - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					} 
					else if(ohif_temp == 2)	//video
					{
						if(load_intensity[net_temp - 1] - increased_load(1, net_temp) + increased_load(h1, net_temp) <= upperbound[net_temp - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
					else //ftp
					{
						if(load_intensity[net_temp - 1] - increased_load(3, net_temp) + increased_load(h1, net_temp) <= upperbound[net_temp - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
				}
				//最終集合產生完了

			}

			else //net1 != 0
			{
				/*--- 對於那些選擇的網路不與mt1相同的mt ---*/
				for(unsigned int onedimind = 0; onedimind < list_of_mt_having_dif_net.size(); onedimind++)
				{
					int ohif_temp = list_of_mt_having_dif_net[onedimind].first;  
					int i_temp = list_of_mt_having_dif_net[onedimind].second;
					int net_temp = solution_before_swap[ohif_temp][i_temp][0].first; 
					
					/*--- 若net2 == 0 分類進list_of_mt_include_mt1_net ---*/
					if(net_temp == 0)
					{
						pair<int, int> net_bs_1 = make_pair(net1, solution_before_swap[vohvif1][i1][0].second); 
						bool k1 = false;
						/*--- check 這些與mt1不同網路的mt2的joinable network set裡是否包含mt1 (有多判斷一次一定不相同的) ---*/
						for(unsigned int net_index = 0; net_index < vohvif_mt_joinable_net_bs_set[ohif_temp][i_temp].size(); net_index ++)
						{
							k1 = compare(net_bs_1, vohvif_mt_joinable_net_bs_set[ohif_temp][i_temp][net_index]);
							if(k1 == true)	//代表網路相同 
							{
								list_of_mt_include_mt1_net.push_back(list_of_mt_having_dif_net[onedimind]);
								break;
							}
						}
					}

					/*--- 若net2 != 0 分類進list_of_mt_both_included_in_joinable_net_set ---*/
					if(net_temp != 0) 
					{
						pair<int, int> net_bs_1 = solution_before_swap[vohvif1][i1][0]; 
						bool k1 = false;
						pair<int, int> net_bs_2 = solution_before_swap[ohif_temp][i_temp][0]; 
						bool k2 = false;
						/*--- check 這些與mt1不同網路的mt的joinable network set裡是否包含mt1 ---*/
						for(unsigned int net_index = 0; net_index < vohvif_mt_joinable_net_bs_set[ohif_temp][i_temp].size(); net_index ++)
						{
							k1 = compare(net_bs_1, vohvif_mt_joinable_net_bs_set[ohif_temp][i_temp][net_index]);
							if(k1 == true)	//代表網路相同 
							{
								/*--- check mt2是否包含在mt1 joinable network set裡面 ---*/
								for(unsigned int mt1_net_index = 0; mt1_net_index < vohvif_mt_joinable_net_bs_set[vohvif1][i1].size(); mt1_net_index ++)
								{
									k2 = compare(net_bs_2, vohvif_mt_joinable_net_bs_set[vohvif1][i1][mt1_net_index]);
									if(k2 == true)	//代表網路相同 
									{
										list_of_mt_both_included_in_joinable_net_set.push_back(list_of_mt_having_dif_net[onedimind]);
										break;
									}
								}
							}	
						}
					}
				}/*--- 把那些選擇的網路不與mt1相同的mt依照網路分類完,產生了兩個集合 ---*/

				/*--- 從第一個集合中挑出滿足loading的mt放入最終集合 ---*/
				for(unsigned int onedimind = 0; onedimind < list_of_mt_include_mt1_net.size(); onedimind++)
				{
					int ohif_temp = list_of_mt_include_mt1_net[onedimind].first;  
					int i_temp = list_of_mt_include_mt1_net[onedimind].second;
					int net_temp = solution_before_swap[ohif_temp][i_temp][0].first;
					if(ohif_temp == 0)	//voice
					{
						if(load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(0, net1) <= upperbound[net1 - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
					else if(ohif_temp == 1)	//http 
					{
						if(load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(2, net1) <= upperbound[net1 - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					} 
					else if(ohif_temp == 2)	//video
					{
						if(load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(1, net1) <= upperbound[net1 - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
					else //ftp
					{
						if(load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(3, net1) <= upperbound[net1 - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
				}
				
				/*--- 從第二個集合中挑出滿足loading的mt放入最終集合 ---*/
				for(unsigned int onedimind = 0; onedimind < list_of_mt_both_included_in_joinable_net_set.size(); onedimind++)
				{
					int ohif_temp = list_of_mt_both_included_in_joinable_net_set[onedimind].first;  
					int i_temp = list_of_mt_both_included_in_joinable_net_set[onedimind].second;
					int net_temp = solution_before_swap[ohif_temp][i_temp][0].first;
					if(ohif_temp == 0)	//voice
					{
						if(load_intensity[net_temp - 1] - increased_load(0, net_temp) + increased_load(h1, net_temp) <= upperbound[net_temp - 1] &&
						   load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(0, net1) <= upperbound[net1 - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
					else if(ohif_temp == 1)	//http 
					{
						if(load_intensity[net_temp - 1] - increased_load(2, net_temp) + increased_load(h1, net_temp) <= upperbound[net_temp - 1] &&
						   load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(2, net1) <= upperbound[net1 - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
					else if(ohif_temp == 2)	//video
					{
						if(load_intensity[net_temp - 1] - increased_load(1, net_temp) + increased_load(h1, net_temp) <= upperbound[net_temp - 1] &&
						   load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(1, net1) <= upperbound[net1 - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
					else //ftp
					{
						if(load_intensity[net_temp - 1] - increased_load(3, net_temp) + increased_load(h1, net_temp) <= upperbound[net_temp - 1] &&
						   load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(3, net1) <= upperbound[net1 - 1])
						{
							list_of_mt2.push_back(make_pair(ohif_temp, i_temp));
						}
					}
				}
				/*--- 最終集合產生完了 ---*/
			}
		
			/*====================================================== test =======================================================*/
// 			if(list_of_mt2.size() != 0)
// 			{
// 				cout << "mt 2 can be one of: " << endl; 
// 				for(unsigned int ind = 0; ind < list_of_mt2.size(); ind++)
// 					cout << "solution_before_swap[" << list_of_mt2[ind].first << "][" << list_of_mt2[ind].second << "][0] " << " net " << solution_before_swap[list_of_mt2[ind].first][list_of_mt2[ind].second][0].first << " bs " 
// 					     << solution_before_swap[list_of_mt2[ind].first][list_of_mt2[ind].second][0].second << endl;
// 				system("PAUSE");
// 			}
			/*===================================================================================================================*/

			/*--- random 從滿足所有條件的mt random選出一個當mt2, 與mt1的網路做交換 ---*/
			if(list_of_mt2.size() != 0)
			{
				/*--- random選 ---*/
				list_index2 = unidrnd(0, list_of_mt2.size() - 1);
				/*--- save information of random selected MT: mt2 ---*/ 
				vohvif2 = list_of_mt2[list_index2].first;
				h2;
				if(vohvif2 == 0 || vohvif2 == 3)
					h2 = vohvif2;
				else if(vohvif1 == 1)
					h2 = 2;
				else	//vohvif2 == 2
					h2 = 1;
				i2 = list_of_mt2[list_index2].second;
				net2 = solution_before_swap[vohvif2][i2][0].first;

				/*==========================================================================================*/
//  				if(net2 != 0)
// 				{
// 					cout << "*2nd selected mt: " << " vohvif " << vohvif2 << " i " << i2 << " net " << net2 
// 						 << ", load_int[" << net2 -1 << "] " << load_intensity[net2 -1] 
// 					     << "- incre load(" << h2 << ", " << net2 << ") " << increased_load(h2, net2) 
// 						 << " = " << load_intensity[net2 -1] - increased_load(h2, net2)  << endl;
// 				}
// 				else
// 					cout << "*2nd selected mt: " << " vohvif " << vohvif2 << " i " << i2 << " net " << net2 << endl;
 				/*==========================================================================================*/

// 				cout << "if change net, net " << net1 << " load_intensity[" << net1 - 1 << "] = " 
// 					 << load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(h2, net1) << endl
// 					 << "net " << net2 << " load_intensity[" << net2 - 1 << "] = " 
// 					 << load_intensity[net2 - 1] - increased_load(h2, net2) + increased_load(h1, net2) << endl;

				/*--- 交換solution_after_swap內的mt1 mt2的網路 ---*/
				swap(solution_after_swap[vohvif1][i1][0], solution_after_swap[vohvif2][i2][0]);
				
				//debug
// 				cout << "i1 " << i1 << " vohvif1 " << vohvif1 << " j1 " << solution_after_swap[vohvif2][i2][0].first 
// 				     << " i2 " << i2 << " vohvif2 " << vohvif2 << " j2 " << solution_after_swap[vohvif1][i1][0].first << endl;
// 				//test
// 				cout << "solution_before_swap V.S solution_after_swap" << endl;
// 				for(unsigned int vohvif = 0; vohvif < solution_before_swap.size(); vohvif++)
// 				{
// 					for(unsigned int i = 0; i < solution_before_swap[vohvif].size(); i++)
// 						cout << "vohvif " << vohvif << " i " << i << " net(before, after) " << solution_before_swap[vohvif][i][0].first << " " << solution_after_swap[vohvif][i][0].first 
// 							 << " bs(before, after) " << solution_before_swap[vohvif][i][0].second << " " << solution_after_swap[vohvif][i][0].second << endl;
// 				}
// 				system("PAUSE");
			}

		   if(list_of_mt2.size() != 0)
		   {
			/*--- 計算新解(交換完)的cost function ---*/		
			
			//group handover blocking factor
			int temp_num_of_blocked_mt_of_new_solution = 0;
			for(unsigned int vohvif =0; vohvif < solution_after_swap.size(); vohvif++)
			{
				for(unsigned int i = 0; i < solution_after_swap[vohvif].size(); i++)
				{
					if(solution_after_swap[vohvif][i][0].first == 0)
						temp_num_of_blocked_mt_of_new_solution++;
				}
			} 
			num_of_blocked_mt_of_new_solution = temp_num_of_blocked_mt_of_new_solution;
			gho_blocking_ratio_of_new_solution = (double)num_of_blocked_mt_of_new_solution / total_num_of_group_mt;

			if(gho_blocking_ratio_of_new_solution > 0.05)
				gho_blocking_factor_of_new_solution = ((gho_blocking_ratio_of_new_solution - 0.05) / (1 - 0.05)) + 3;
			else
				gho_blocking_factor_of_new_solution = gho_blocking_ratio_of_new_solution;

			/*==========================================================================================*/
//			cout << "num_of_blocked_mt_of_new_solution " << num_of_blocked_mt_of_new_solution << endl;
//			system("PAUSE");
			/*==========================================================================================*/

			//dwelling time factor (改變)				
			for(unsigned int vohvif = 0; vohvif < solution_after_swap.size(); vohvif++)
			{
				for(unsigned int i = 0; i < solution_after_swap[vohvif].size(); i++)
				{
					if(solution_after_swap[vohvif][i][0].first != 0)
					{
						int j = solution_after_swap[vohvif][i][0].first;
						after_swap_dwelling_time_ratio[vohvif][i][0] = min(dwelling_time_expand[j - 1] / bus.alias_of_mt_container()[list[vohvif][i]].get_life_time(), 1.0); 
						sum_of_dwelling_time_ratio_of_new_solution += after_swap_dwelling_time_ratio[vohvif][i][0];
					}
				}
			}

			average_dwelling_time_ratio_of_new_solution = sum_of_dwelling_time_ratio_of_new_solution / (total_num_of_group_mt - num_of_blocked_mt_of_new_solution);
			double temp_sum = 0;
			
			for(unsigned int vohvif = 0; vohvif < solution_after_swap.size(); vohvif++)
			{	
				for(unsigned int i = 0; i < solution_after_swap[vohvif].size(); i++)
				{
					if(solution_after_swap[vohvif][i][0].first != 0)
						temp_sum += pow(after_swap_dwelling_time_ratio[vohvif][i][0] - average_dwelling_time_ratio_of_new_solution, 2);
				}
			}

			variance_of_dwelling_time_ratio_of_new_solution = sqrt(temp_sum / (total_num_of_group_mt - num_of_blocked_mt_of_new_solution)); 
			dwelling_time_factor_of_new_solution = exp(-average_dwelling_time_ratio_of_new_solution + variance_of_dwelling_time_ratio_of_new_solution);
			
			//load balancing factor
			after_swap_load_intensity = load_intensity;

			//======================================= debug ====================================================
// 			cout << "h1 " << h1 << ", net1 " << net1 << "; h2 " << h2 << ", net2 " << net2 << endl;
// 
// 			cout << "load intensity before swap " << after_swap_load_intensity[0] << " " << after_swap_load_intensity[1] << " " 
// 				                                  << after_swap_load_intensity[2] << " " << after_swap_load_intensity[3] << endl;
			//==================================================================================================

			if(net1 != 0 && net2 != 0)
			{
				after_swap_load_intensity[net1 - 1] = load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(h2, net1);
				after_swap_load_intensity[net2 - 1] = load_intensity[net2 - 1] - increased_load(h2, net2) + increased_load(h1, net2);
			}
			else if(net1 != 0 && net2 == 0)
				after_swap_load_intensity[net1 - 1] = load_intensity[net1 - 1] - increased_load(h1, net1) + increased_load(h2, net1);
				
			else if(net1 == 0 && net2 != 0)
				after_swap_load_intensity[net2 - 1] = load_intensity[net2 - 1] - increased_load(h2, net2) + increased_load(h1, net2);

			double temp_loading_sum = 0, temp_average_square_sum = 0;
			for(int h = 0; h < 4; h++)
				temp_loading_sum += after_swap_load_intensity[h];
			average_load_intensity_of_new_solution = temp_loading_sum / 4;
			for(int h = 0; h < 4; h++)
				temp_average_square_sum += pow(after_swap_load_intensity[h] - average_load_intensity_of_new_solution, 2);
			load_balancing_factor_of_new_solution = sqrt(temp_average_square_sum / 4);
			
			//======================================== debug ===============================================
// 			cout << "load intensity after swap " << after_swap_load_intensity[0] << " " << after_swap_load_intensity[1] << " "
// 				                                 << after_swap_load_intensity[2] << " " << after_swap_load_intensity[3] << endl;
// 			system("PAUSE");
			//==============================================================================================
			
			//QoS factor
			/*--- 先找到非0的網路的mt---*//*--- QoS degree全部重算 ---*/
			for(unsigned int vohvif = 0; vohvif < solution_after_swap.size(); vohvif++)			
			{
				for(unsigned int i = 0; i < solution_after_swap[vohvif].size(); i++)
				{
					if(solution_after_swap[vohvif][i][0].first != 0)
					{
						if(vohvif == 0)
						{
							if(solution_after_swap[vohvif][i][0].first == 1)
							{
								double QoS_degree = (delay_constraint[0] - network_average_delay[0][0]) / delay_constraint[0];
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
							else if(solution_after_swap[vohvif][i][0].first == 3)
							{
								double QoS_degree = (delay_constraint[0] - network_average_delay[2][0]) / delay_constraint[0];
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
							else if(solution_after_swap[vohvif][i][0].first == 4)
							{
								double QoS_degree = (delay_constraint[0] - network_average_delay[3][0]) / delay_constraint[0];
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
						}
						else if(vohvif == 1)
						{
							if(solution_after_swap[vohvif][i][0].first == 2)
							{
								double QoS_degree = min((net_avg_trans_rate[0][0] / (10.0 * min_trans_rate[0])), 1.0);
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
							else if(solution_after_swap[vohvif][i][0].first == 3)
							{
								double QoS_degree = min((net_avg_trans_rate[1][0] / (10.0 * min_trans_rate[0])), 1.0);
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
							else if(solution_after_swap[vohvif][i][0].first == 4)
							{
								double QoS_degree = min((net_avg_trans_rate[2][0] / (10.0 * min_trans_rate[0])), 1.0);
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}	
						}
						else if(vohvif == 2)
						{
							if(solution_after_swap[vohvif][i][0].first == 2)
							{
								double QoS_degree = (delay_constraint[1] - network_average_delay[1][0]) / delay_constraint[1];
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
							else if(solution_after_swap[vohvif][i][0].first == 3)
							{
								double QoS_degree = (delay_constraint[1] - network_average_delay[2][1]) / delay_constraint[1];
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
							else if(solution_after_swap[vohvif][i][0].first == 4)
							{
								double QoS_degree = (delay_constraint[1] - network_average_delay[3][1]) / delay_constraint[1];
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
						}
						else
						{
							if(solution_after_swap[vohvif][i][0].first == 2)
							{
								double QoS_degree = min((net_avg_trans_rate[0][1] / (10.0 * min_trans_rate[1])), 1.0);
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
							else if(solution_after_swap[vohvif][i][0].first == 3)
							{
								double QoS_degree = min((net_avg_trans_rate[1][1] / (10.0 * min_trans_rate[1])), 1.0);
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}
							else if(solution_after_swap[vohvif][i][0].first == 4)
							{
								double QoS_degree = min((net_avg_trans_rate[2][1] / (10.0 * min_trans_rate[1])), 1.0);
								after_swap_QoS_factor[vohvif][i][0] = 1 - ( 1 / ( 1 + 100 * exp(-10 * QoS_degree)));
							}	
						}
						sum_of_QoS_factor_of_new_solution += after_swap_QoS_factor[vohvif][i][0];
					}
				}
			}
			QoS_factor_of_new_solution = sum_of_QoS_factor_of_new_solution / (total_num_of_group_mt - num_of_blocked_mt_of_new_solution);		
			
			//cost function
			cost_function_value_of_new_solution = gho_blocking_factor_of_new_solution + dwelling_time_factor_of_new_solution + load_balancing_factor_of_new_solution + QoS_factor_of_new_solution;
			
			/*=====================================================================================================*/
			cout << "T: " << T << " l: " << l << endl;
			cout << "**gho_blocking_factor_of_new_solution: " << gho_blocking_factor_of_new_solution << endl;
			cout << "**dwelling_time_factor_of_new_solution: " << dwelling_time_factor_of_new_solution << endl;
			cout << "**load_balancing_factor_of_new_solution: " << load_balancing_factor_of_new_solution << endl;
			cout << "**QoS_factor_of_new_solution: " << QoS_factor_of_new_solution << endl;
 			cout << "**cost_function_value_of_new_solution: " << cost_function_value_of_new_solution << endl;
			cout << "----------------------------------------------------" << endl;
//			system("PAUSE");
			/*=====================================================================================================*/

			/*--- 與舊解的cost function相減 ---*/
			double delta_f = cost_function_value_of_new_solution - cost_function_value_of_ini_solution;

			//test
// 			cout << "delta_f " << delta_f << " exp(-delta_f / T) " << exp(-delta_f / T) << endl;
// 			system("PAUSE");

			if(delta_f < 0)	
			{
				/*--- 最佳解被新解取代,舊解被新解取代, 要更新舊解很多參數, 新解參數歸零 ---*/
				cost_function_value_of_ini_solution = cost_function_value_of_new_solution;
				solution_before_swap = solution_after_swap;
				load_intensity = after_swap_load_intensity;

				num_of_blocked_mt_of_ini_solution = num_of_blocked_mt_of_new_solution;

				load_intensity_of_best_solution = after_swap_load_intensity;
				best_solution = solution_after_swap;
				num_of_blocked_mt_of_best_solution = num_of_blocked_mt_of_new_solution;
				
				T_best = T;
				l_best = l;
				gho_blocking_factor_best = gho_blocking_factor_of_new_solution;
				dwelling_time_factor_best = dwelling_time_factor_of_new_solution;
				load_balancing_factor_best = load_balancing_factor_of_new_solution;
				QoS_factor_best = QoS_factor_of_new_solution;
				cost_function_best = cost_function_value_of_new_solution;	
			}
			else
			{
				/*--- 舊解被新解取代 ---*/
				if(exp(-delta_f / T) > unifrnd(0, 1))
				{
					cost_function_value_of_ini_solution = cost_function_value_of_new_solution;
					solution_before_swap = solution_after_swap;
					load_intensity = after_swap_load_intensity;

					num_of_blocked_mt_of_ini_solution = num_of_blocked_mt_of_new_solution;
				}
				/*--- 舊解不被新解取代 ---*/
				else 
				{
					/*--- solution_after_swap內的值要swap回舊解時的值 ---*/ //才不會load沒換網路卻換了
					swap(solution_after_swap[vohvif1][i1][0], solution_after_swap[vohvif2][i2][0]);	
					c++;	
				}
			}
		   }
		}
		if(c >= C ||  T * gamma < Tmin)
		{
			/*--- 輸出解 ---*//*--- assign新網路, bs ---*/
			for(unsigned int vohvif = 0; vohvif < best_solution.size(); vohvif ++)
			{
				for(unsigned int i = 0; i < best_solution[vohvif].size(); i++)
				{
					bus.alias_of_mt_container()[list[vohvif][i]].get_network_bs_number() = best_solution[vohvif][i][0];
				}	
			}
			/*--- 結束SA ---*/
			num_of_blocked_mt = num_of_blocked_mt_of_best_solution; 
			
// 			cout << "c　" << c << " T " << T << " num_of_blocked_mt " << endl;
// 			system("PAUSE");

			/*==========================================================================*/
			cout << "-------------------------------------------------------------------"<< endl;
			cout << "T_best: " << T_best << " l_best: " << l_best << endl;
			cout << "gho_blocking_factor_of_new_solution " << gho_blocking_factor_best << endl;
			cout << "dwelling_time_factor_best: " << dwelling_time_factor_best << endl;
			cout << "load_balancing_factor_best: " << load_balancing_factor_best << endl;
			cout << "QoS_factor_best: " << QoS_factor_best << endl;
			cout << "cost_function_best: " << cost_function_best << endl; 
			cout << "-------------------------------------------------------------------"<< endl;
			/*==========================================================================*/

			goto anchor;
		}
		T = T * gamma;
	}
	anchor:
	/*--- 更新main()的load intensity ---*/
	gsm_load_intensity = load_intensity_of_best_solution[0];
	edge_load_intensity = load_intensity_of_best_solution[1];
	wcdma_load_intensity = load_intensity_of_best_solution[2];
	lte_load_intensity = load_intensity_of_best_solution[3];

	//test
	cout << "After GHO, load intensity: " << gsm_load_intensity << " " << edge_load_intensity << " " << wcdma_load_intensity << " " << lte_load_intensity << endl;
	cout << "num_of_blocked_mt_of_best_solu　" << num_of_blocked_mt << endl;
	cout << "==================== SAGHOD-optimal END =========================" << endl;
//	system("PAUSE");
 }