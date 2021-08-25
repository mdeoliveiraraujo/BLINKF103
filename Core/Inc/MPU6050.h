// Header for the MPU6050 communication

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define RAD_2_DEG             57.29578 // [deg/rad]
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0

#define DEFAULT_GYRO_COEFF    0.98

void MPU6050_Init (void);
void MPU6050_Read_Accel (void);
void MPU6050_Read_Gyro (void);
void MPU6050_Retrieve_Data (void);

void setGyroOffsets(float x, float y, float z);
void setAccOffsets(float x, float y, float z);
void setFilterGyroCoef(float gyro_coeff);
void setFilterAccCoef(float acc_coeff);
void calcOffsets(int is_calc_gyro, int is_calc_acc);
void MPU6050_Update (void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
