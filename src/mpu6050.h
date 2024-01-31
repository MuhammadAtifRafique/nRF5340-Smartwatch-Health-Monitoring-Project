#ifndef MPU6050_H__
#define MPU6050_H__

#include <stdint.h>
#include <stdbool.h>
////////////////| For testing Step Counter Algo |////////////////////
#define MPU6050_REGISTER_GYRO_CONFIG        (0x1B)
#define MPU6050_GCONFIG_FS_SEL_LENGTH       (2)
#define MPU6050_GCONFIG_FS_SEL_BIT          (4)
#define MPU6050_REGISTER_ACCEL_CONFIG       (0x1C)
#define MPU6050_ACONFIG_AFS_SEL_BIT         (4)
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      (2)
#define MPU6050_REGISTER_ACCEL_XOUT_H       (0x3B)
#define MPU6050_INTERRUPT_DMP_INT_BIT       (1)
#define MPU6050_REGISTER_INT_STATUS         (0x3A)
#define AVG_BUFF_SIZE   20
typedef struct _mpu6050_acceleration_t
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} mpu6050_acceleration_t;
typedef struct _mpu6050_rotation_t
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_rotation_t;
void esp32_i2c_read_bit
(
	uint8_t register_address,
	uint8_t bit_number,
	uint8_t* data
);
void esp32_i2c_read_bit(uint8_t register_address,uint8_t bit_number,uint8_t* data);
void esp32_i2c_read_bits(uint8_t register_address,uint8_t bit_start,uint8_t size,uint8_t* data);
bool mpu6050_get_int_dmp_status();
void mpu6050_get_acceleration(mpu6050_acceleration_t* data);
uint8_t mpu6050_get_full_scale_accel_range();
float mpu6050_get_accel_res(uint8_t accel_scale);
void mpu6050_get_rotation(mpu6050_rotation_t* data);
uint8_t mpu6050_get_full_scale_gyro_range();
void mpu6050_madgwick_quaternion_update
(float accel_x,float accel_y,float accel_z,float gyro_x,float gyro_y,float gyro_z);
const char* mpu6050_get_tag();
void step_counter(void);
void init_mpu6050(void);
void update_calories(int steps);
//////////////////| End of Step Counter Algo |///////////////////////
#define MPU6050_ADDRESS_LEN  1         //MPU6050
#define MPU6050_ADDRESS     (0xD0>>1)  //MPU6050 Device Address
#define MPU6050_WHO_AM_I     0x68U     //MPU6050 ID


#define MPU6050_GYRO_OUT        0x43
#define MPU6050_ACC_OUT         0x3B

#define ADDRESS_WHO_AM_I          (0x75U) //  WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0x68U) // 

//MPU6050 Registers addresses, see datasheet for more info and each register's function
#define MPU_SELF_TESTX_REG		0x0D	
#define MPU_SELF_TESTY_REG		0x0E	
#define MPU_SELF_TESTZ_REG		0x0F	
#define MPU_SELF_TESTA_REG		0x10	
#define MPU_SAMPLE_RATE_REG		0x19	
#define MPU_CFG_REG                     0x1A	
#define MPU_GYRO_CFG_REG		0x1B	
#define MPU_ACCEL_CFG_REG		0x1C	
#define MPU_MOTION_DET_REG		0x1F	
#define MPU_FIFO_EN_REG			0x23	
#define MPU_I2CMST_CTRL_REG		0x24	
#define MPU_I2CSLV0_ADDR_REG            0x25	
#define MPU_I2CSLV0_REG			0x26	
#define MPU_I2CSLV0_CTRL_REG            0x27	
#define MPU_I2CSLV1_ADDR_REG            0x28	
#define MPU_I2CSLV1_REG			0x29	
#define MPU_I2CSLV1_CTRL_REG            0x2A	
#define MPU_I2CSLV2_ADDR_REG            0x2B	
#define MPU_I2CSLV2_REG			0x2C	
#define MPU_I2CSLV2_CTRL_REG            0x2D	
#define MPU_I2CSLV3_ADDR_REG            0x2E	
#define MPU_I2CSLV3_REG			0x2F	
#define MPU_I2CSLV3_CTRL_REG            0x30	
#define MPU_I2CSLV4_ADDR_REG            0x31	
#define MPU_I2CSLV4_REG			0x32	
#define MPU_I2CSLV4_DO_REG		0x33	
#define MPU_I2CSLV4_CTRL_REG            0x34	
#define MPU_I2CSLV4_DI_REG		0x35	


#define MPU_PWR_MGMT1_REG		0x6B	
#define MPU_PWR_MGMT2_REG		0x6C	

#define MPU_I2CMST_STA_REG		0x36	
#define MPU_INTBP_CFG_REG		0x37	
#define MPU_INT_EN_REG			0x38	
#define MPU_INT_STA_REG			0x3A	

#define MPU_I2CMST_DELAY_REG            0x67	
#define MPU_SIGPATH_RST_REG		0x68	
#define MPU_MDETECT_CTRL_REG            0x69	
#define MPU_USER_CTRL_REG		0x6A	
#define MPU_PWR_MGMT1_REG		0x6B	
#define MPU_PWR_MGMT2_REG		0x6C	
#define MPU_FIFO_CNTH_REG		0x72	
#define MPU_FIFO_CNTL_REG		0x73	
#define MPU_FIFO_RW_REG			0x74	
#define MPU_DEVICE_ID_REG		0x75	

bool mpu6050_init(void);    // initialize the mpu6050
bool mpu6050_begin(void);

/**
  @brief Function for writing a MPU6050 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool mpu6050_register_write(uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading MPU6050 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool mpu6050_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying MPU6050 product ID.
  @retval true Product ID is what was expected
  @retval false Product ID was not what was expected
*/
bool mpu6050_verify_product_id(void);


bool MPU6050_ReadGyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z );
bool MPU6050_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z );
bool awakeCheck(int16_t zeeValue);

#endif


