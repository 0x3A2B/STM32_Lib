/**
 * @file testbench.c
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */

float fun1(float input){
	static float output[10];
	static float u[10];
	
	output[2] = output[1];output[1] = output[0];
	u[2] = u[1];u[1] = u[0]; u[0] = input;
	output[0] = 0.368 * output[1] + 0.26 *output[2] + 0.1 * u[1] + 0.632 * u[2];
	
	return output[0];
}
float fun2(float input){
   static float output[10];
	static float u[10];
   int i;
   u[0] = input;
   for(i = 3;i>0;i--){
      output[i] = output[i - 1];
      u[i] = u[i - 1];
   }

   output[0] = (- 0.3 * output[1] + u[1]) / (6 + output[1]*output[1]);

   return output[0];
}
float moter2(float input){
	static float output[10];
	static float u[10];
	int k = 3;
	static float den[10] = {1,-2.90633930670106,2.82269562149066,-0.916356314789603};
	static float num[10] = {0,0.0000853331265147964,0.000333831370206075,0.0000816859077063558};
	
	int i = 0;
	for(i = k;i >= 0;i--){
		output[i] = output[i-1];
		u[i] = u[i - 1];
	}
	u[0] = input;
	
	output[0] = num[1]*u[0] + num[2]*u[1] + num[3]*u[2]- \
							 den[1]*output[1] - den[2]*output[2] - den[3]*output[3];
	
	return output[0];
}

float moter(float input){
	static float output[10];
	static float u[10];
	int k = 2;
	static float den[10] = {1,-1.97530991202833, 0.975309912028333, -0.916356314789603};
	static float num[10] = {0,0.000065949,0.000065401988};
	
	int i = 0;
	for(i = k;i >= 0;i--){
		output[i] = output[i-1];
		u[i] = u[i - 1];
	}
	u[0] = input;
	
	output[0] = num[1]*u[0] + num[2]*u[1] - den[1]*output[1] - den[2]*output[2];
	
	//if(output[0] > 2) output[0]=2;
	//if(output[0] < 0)output[0]=0;
	return output[0];
}
