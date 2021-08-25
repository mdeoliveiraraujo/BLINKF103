#include "MPU6050.h"
#include "main.h"
#include "math.h"
#include "stdio.h"
#include <string.h>
#include "file_handling.h"
#include "stm32f1xx_it.h"
#include "ads1256.h"

unsigned long preInterval;

int upsideDownMounting = 0;

int16_t Accel_X_RAW;
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;
int16_t Temperature;
int16_t Gyro_X_RAW;
int16_t Gyro_Y_RAW;
int16_t Gyro_Z_RAW;

float angleAccX, angleAccY;
float angleX, angleY, angleZ;

float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;

float acc_lsb_to_g = 16384.0;
float gyro_lsb_to_degsec = 65.5;

float accXoffset, accYoffset, accZoffset;
float gyroXoffset, gyroYoffset, gyroZoffset;

float filterGyroCoef = DEFAULT_GYRO_COEFF;

float acc_total_vector, angle_pitch_acc, angle_roll_acc;

int is_calc_acc = 1;
int is_calc_gyro = 1;

I2C_HandleTypeDef hi2c1;

void MPU6050_Init (void)
{
	uint8_t check, Data;
	char uart_buffer[64];
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	sprintf(uart_buffer,"check: %lu\n",(unsigned long)check);
	send_uart(uart_buffer);
	if (check == 114)
	{
		send_uart("Configurando a IMU...\n");

		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		Data = 0x08;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
	send_uart("Calculando offsets da IMU.\n");
	calcOffsets(1,1);
	MPU6050_Update();
	preInterval = UptimeMillis/1000;

}

void MPU6050_Retrieve_Data (void)
{
	uint8_t Rec_Data[14];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	Temperature = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

	Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	acc_total_vector = sqrt((Accel_X_RAW*Accel_X_RAW)+(Accel_Y_RAW*Accel_Y_RAW)+(Accel_Z_RAW*Accel_Z_RAW));
	angle_pitch_acc = (Accel_X_RAW/16384.0-accXoffset)*90;//asin((float)Accel_Y_RAW/acc_total_vector)*57.296;
	angle_roll_acc = (Accel_Y_RAW/16384.0-accYoffset)*90; //asin((float)Accel_X_RAW/acc_total_vector)*(-57.296);

	accX = ((float)Accel_X_RAW) / acc_lsb_to_g - accXoffset;
	accY = ((float)Accel_Y_RAW) / acc_lsb_to_g - accYoffset;
	accZ = (!upsideDownMounting - upsideDownMounting) * ((float)Accel_Z_RAW) / acc_lsb_to_g - accZoffset;
	temp = (Temperature + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
	gyroX = ((float)Gyro_X_RAW) / gyro_lsb_to_degsec - gyroXoffset;
	gyroY = ((float)Gyro_Y_RAW) / gyro_lsb_to_degsec - gyroYoffset;
	gyroZ = ((float)Gyro_Z_RAW) / gyro_lsb_to_degsec - gyroZoffset;
}

void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle,float limit){
	while (angle >  limit) angle -= 2*limit;
	while (angle < -limit) angle += 2*limit;
	return angle;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setGyroOffsets(float x, float y, float z){
	gyroXoffset = x;
	gyroYoffset = y;
	gyroZoffset = z;
}

void setAccOffsets(float x, float y, float z){
	accXoffset = x;
	accYoffset = y;
	accZoffset = z;
}

void setFilterGyroCoef(float gyro_coeff){
	if ((gyro_coeff < 0) | (gyro_coeff > 1))
	{
		gyro_coeff = DEFAULT_GYRO_COEFF;
	} // prevent bad gyro coeff, should throw an error...
	filterGyroCoef = gyro_coeff;
}

void setFilterAccCoef(float acc_coeff)
{
	setFilterGyroCoef(1.0 - acc_coeff);
}

void calcOffsets(int is_calc_gyro, int is_calc_acc){
	if(is_calc_gyro)
	{
		setGyroOffsets(0,0,0);
	}

	if(is_calc_acc)
	{
		setAccOffsets(0,0,0);
	}
	float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro

	for(int i = 0; i < CALIB_OFFSET_NB_MES; i++)
	{
		MPU6050_Retrieve_Data();
		ag[0] += accX;
		ag[1] += accY;
		ag[2] += (accZ-1.0);
		ag[3] += gyroX;
		ag[4] += gyroY;
		ag[5] += gyroZ;
		HAL_Delay(1); // wait a little bit between 2 measurements
	}

	if(is_calc_acc)
	{
		accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
		accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
		accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
	}

	if(is_calc_gyro)
	{
		gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
		gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
		gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
	}
}

void MPU6050_Update (void)
{
	MPU6050_Retrieve_Data();	// Retrieve data from MPU6050 (Acc, Temp, Gyro)

	// estimate tilt angles: this is an approximation for small angles!
	float sgZ = (accZ>=0)-(accZ<0); // allow one angle to go from -180 to +180 degrees
	angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180,+180] deg
	angleAccY = - atan2(accX,     sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG; // [- 90,+ 90] deg

	unsigned long Tnew = UptimeMillis;
	float dt = (Tnew - preInterval)/1000.0;
	preInterval = Tnew;

	angleX = wrap(filterGyroCoef*(angleAccX + wrap(angleX +     gyroX*dt - angleAccX,180)) + (1.0-filterGyroCoef)*angleAccX,180);
	angleY = wrap(filterGyroCoef*(angleAccY + wrap(angleY + sgZ*gyroY*dt - angleAccY, 90)) + (1.0-filterGyroCoef)*angleAccY, 90);
	angleZ += gyroZ*dt; // not wrapped (to do???)
}
