#ifndef __MAHONY_H
#define __MAHONY_H



extern volatile float q0, q1, q2, q3; 


void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif
