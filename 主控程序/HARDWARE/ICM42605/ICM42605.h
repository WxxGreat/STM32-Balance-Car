/*
 * ICM42605.h
 *
 *  Created on: 2022年3月7日
 *      Author: 
 */

#ifndef CODE_ICM42605_H_
#define CODE_ICM42605_H_



// Bank 0
#define ICM42605_DEVICE_CONFIG             0x11
#define ICM42605_DRIVE_CONFIG              0x13
#define ICM42605_INT_CONFIG                0x14
#define ICM42605_FIFO_CONFIG               0x16
#define ICM42605_TEMP_DATA1                0x1D
#define ICM42605_TEMP_DATA0                0x1E
#define ICM42605_ACCEL_DATA_X1             0x1F
#define ICM42605_ACCEL_DATA_X0             0x20
#define ICM42605_ACCEL_DATA_Y1             0x21
#define ICM42605_ACCEL_DATA_Y0             0x22
#define ICM42605_ACCEL_DATA_Z1             0x23
#define ICM42605_ACCEL_DATA_Z0             0x24
#define ICM42605_GYRO_DATA_X1              0x25
#define ICM42605_GYRO_DATA_X0              0x26
#define ICM42605_GYRO_DATA_Y1              0x27
#define ICM42605_GYRO_DATA_Y0              0x28
#define ICM42605_GYRO_DATA_Z1              0x29
#define ICM42605_GYRO_DATA_Z0              0x2A
#define ICM42605_TMST_FSYNCH               0x2B
#define ICM42605_TMST_FSYNCL               0x2C
#define ICM42605_INT_STATUS                0x2D
#define ICM42605_FIFO_COUNTH               0x2E
#define ICM42605_FIFO_COUNTL               0x2F
#define ICM42605_FIFO_DATA                 0x30
#define ICM42605_APEX_DATA0                0x31
#define ICM42605_APEX_DATA1                0x32
#define ICM42605_APEX_DATA2                0x33
#define ICM42605_APEX_DATA3                0x34
#define ICM42605_APEX_DATA4                0x35
#define ICM42605_APEX_DATA5                0x36
#define ICM42605_INT_STATUS2               0x37
#define ICM42605_INT_STATUS3               0x38
#define ICM42605_SIGNAL_PATH_RESET         0x4B
#define ICM42605_INTF_CONFIG0              0x4C
#define ICM42605_INTF_CONFIG1              0x4D
#define ICM42605_PWR_MGMT0                 0x4E
#define ICM42605_GYRO_CONFIG0              0x4F
#define ICM42605_ACCEL_CONFIG0             0x50
#define ICM42605_GYRO_CONFIG1              0x51
#define ICM42605_GYRO_ACCEL_CONFIG0        0x52
#define ICM42605_ACCEL_CONFIG1             0x53
#define ICM42605_TMST_CONFIG               0x54
#define ICM42605_APEX_CONFIG0              0x56
#define ICM42605_SMD_CONFIG                0x57
#define ICM42605_FIFO_CONFIG1              0x5F
#define ICM42605_FIFO_CONFIG2              0x60
#define ICM42605_FIFO_CONFIG3              0x61
#define ICM42605_FSYNC_CONFIG              0x62
#define ICM42605_INT_CONFIG0               0x63
#define ICM42605_INT_CONFIG1               0x64
#define ICM42605_INT_SOURCE0               0x65
#define ICM42605_INT_SOURCE1               0x66
#define ICM42605_INT_SOURCE3               0x68
#define ICM42605_INT_SOURCE4               0x69
#define ICM42605_FIFO_LOST_PKT0            0x6C
#define ICM42605_FIFO_LOST_PKT1            0x6D
#define ICM42605_SELF_TEST_CONFIG          0x70
#define ICM42605_WHO_AM_I                  0x75
#define ICM42605_REG_BANK_SEL              0x76

// Bank 1
#define ICM42605_SENSOR_CONFIG0            0x03
#define ICM42605_GYRO_CONFIG_STATIC2       0x0B
#define ICM42605_GYRO_CONFIG_STATIC3       0x0C
#define ICM42605_GYRO_CONFIG_STATIC4       0x0D
#define ICM42605_GYRO_CONFIG_STATIC5       0x0E
#define ICM42605_GYRO_CONFIG_STATIC6       0x0F
#define ICM42605_GYRO_CONFIG_STATIC7       0x10
#define ICM42605_GYRO_CONFIG_STATIC8       0x11
#define ICM42605_GYRO_CONFIG_STATIC9       0x12
#define ICM42605_GYRO_CONFIG_STATIC10      0x13
#define ICM42605_XG_ST_DATA                0x5F
#define ICM42605_YG_ST_DATA                0x60
#define ICM42605_ZG_ST_DATA                0x61
#define ICM42605_TMSTVAL0                  0x62
#define ICM42605_TMSTVAL1                  0x63
#define ICM42605_TMSTVAL2                  0x64
#define ICM42605_INTF_CONFIG4              0x7A
#define ICM42605_INTF_CONFIG5              0x7B
#define ICM42605_INTF_CONFIG6              0x7C

// Bank 2
#define ICM42605_ACCEL_CONFIG_STATIC2      0x03
#define ICM42605_ACCEL_CONFIG_STATIC3      0x04
#define ICM42605_ACCEL_CONFIG_STATIC4      0x05
#define ICM42605_XA_ST_DATA                0x3B
#define ICM42605_YA_ST_DATA                0x3C
#define ICM42605_ZA_ST_DATA                0x3D

// Bank 4
#define ICM42605_GYRO_ON_OFF_CONFIG        0x0E
#define ICM42605_APEX_CONFIG1              0x40
#define ICM42605_APEX_CONFIG2              0x41
#define ICM42605_APEX_CONFIG3              0x42
#define ICM42605_APEX_CONFIG4              0x43
#define ICM42605_APEX_CONFIG5              0x44
#define ICM42605_APEX_CONFIG6              0x45
#define ICM42605_APEX_CONFIG7              0x46
#define ICM42605_APEX_CONFIG8              0x47
#define ICM42605_APEX_CONFIG9              0x48
#define ICM42605_ACCEL_WOM_X_THR           0x4A
#define ICM42605_ACCEL_WOM_Y_THR           0x4B
#define ICM42605_ACCEL_WOM_Z_THR           0x4C
#define ICM42605_INT_SOURCE6               0x4D
#define ICM42605_INT_SOURCE7               0x4E
#define ICM42605_INT_SOURCE8               0x4F
#define ICM42605_INT_SOURCE9               0x50
#define ICM42605_INT_SOURCE10              0x51
#define ICM42605_OFFSET_USER0              0x77
#define ICM42605_OFFSET_USER1              0x78
#define ICM42605_OFFSET_USER2              0x79
#define ICM42605_OFFSET_USER3              0x7A
#define ICM42605_OFFSET_USER4              0x7B
#define ICM42605_OFFSET_USER5              0x7C
#define ICM42605_OFFSET_USER6              0x7D
#define ICM42605_OFFSET_USER7              0x7E
#define ICM42605_OFFSET_USER8              0x7F

#define ICM42605_ADDRESS                   0x69   // Address of ICM42605 accel/gyro when ADO = HIGH
                                                  //SA0接地：0x68   SA0上拉：0x69  模块默认上拉
#define ICM42605_SPI_W                     0x00
#define ICM42605_SPI_R                     0x80


#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00  // default

#define GFS_2000DPS   0x00 // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_5DPS   0x05
#define GFS_31_25DPS  0x06
#define GFS_15_125DPS 0x07

#define AODR_8000Hz   0x03
#define AODR_4000Hz   0x04
#define AODR_2000Hz   0x05
#define AODR_1000Hz   0x06 // default
#define AODR_200Hz    0x07
#define AODR_100Hz    0x08
#define AODR_50Hz     0x09
#define AODR_25Hz     0x0A
#define AODR_12_5Hz   0x0B
#define AODR_6_25Hz   0x0C
#define AODR_3_125Hz  0x0D
#define AODR_1_5625Hz 0x0E
#define AODR_500Hz    0x0F

#define GODR_8000Hz  0x03
#define GODR_4000Hz  0x04
#define GODR_2000Hz  0x05
#define GODR_1000Hz  0x06 // default
#define GODR_200Hz   0x07
#define GODR_100Hz   0x08
#define GODR_50Hz    0x09
#define GODR_25Hz    0x0A
#define GODR_12_5Hz  0x0B
#define GODR_500Hz   0x0F


extern signed short int icm4_temp;




void icm42605_AD_trun(void);
//--------软件IIC--------------
char ICM42605_init(void);
void ICM42605_reset(void);
void Get_ICM42605_Tempdata(float* ICM42605_Temperature);
void Get_ICM42605_accdata(short *ax, short *ay, short *az);
void Get_ICM42605_gyro(short *gx, short *gy, short *gz);

unsigned char ICM42605_Write_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
unsigned char ICM42605_Read_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
unsigned char ICM42605_Write_Byte(unsigned char reg, unsigned char data);
unsigned char ICM42605_Read_Byte(unsigned char reg);


#endif /* CODE_ICM42605_H_ */
