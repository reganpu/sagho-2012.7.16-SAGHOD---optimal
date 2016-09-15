#ifndef PATH_LOSS_H_
#define PATH_LOSS_H_

double gsm_edge_path_loss(double d); //GSM/EDGE path loss 
double wcdma_path_loss(double d);	 //WCDMA path loss 
double lte_path_loss(double d);		 //LTE path loss

double path_loss(int j, double d);

#endif 

