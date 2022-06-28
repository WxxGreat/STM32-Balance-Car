#ifndef __KALMAN_H
#define __KALMAN_H 

//卡尔曼解算法库


extern float Angle_Y_Final;			//解算后横滚角


void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);


#endif
