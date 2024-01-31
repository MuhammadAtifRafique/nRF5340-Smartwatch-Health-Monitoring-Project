#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include "sqrt.h"
#include "mpu6050.h"
#include "TimeStamp.h"
/////////////// | Start calories | //////////
float calories = 0.0; // insert calories (in cal unit) from update_calories function
/////////////// | End calories | //////////
///////////| Start of step counter |///////////////////
bool step_counter_start = true;
uint8_t step_buffer[14];
float accel_bias[3] = {0, 0, 0};
float gyro_bias[3] = {0, 0, 0};
const char *TAG_MPU6050 = "MPU6050";
float quart[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float delta_t = 0.0f;
#define PI              (3.14159265358979323846f)
#define GYRO_MEAS_ERROR (PI * (60.0f / 180.0f))
#define GYRO_MEAS_DRIFT (PI * (1.0f / 180.0f))
#define BETA            (sqrt(3.0f / 4.0f) * GYRO_MEAS_ERROR)
#define ZETA            (sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT)

///
mpu6050_acceleration_t accel;
    mpu6050_rotation_t gyro;
    uint64_t now_time = 0, prev_time = 0;
    int8_t range = 0;
    float accel_g_x, accel_g_y, accel_g_z;
    float accel_int_x, accel_int_y, accel_int_z;
    float gyro_ds_x, gyro_ds_y, gyro_ds_z, accel_res, gyro_res;
    int accel_x_avg_buff[AVG_BUFF_SIZE];
    int accel_y_avg_buff[AVG_BUFF_SIZE];
    int accel_z_avg_buff[AVG_BUFF_SIZE];
    int accel_x_avg_buff_count = 0;
    int accel_y_avg_buff_count = 0;
    int accel_z_avg_buff_count = 0;
    int accel_x_avg, accel_y_avg, accel_z_avg;
    int min_reg_accel_x = 0, min_reg_accel_y = 0, min_reg_accel_z = 0;
    int max_reg_accel_x = 0, max_reg_accel_y = 0, max_reg_accel_z = 0;
    int min_curr_accel_x, min_curr_accel_y, min_curr_accel_z;
    int max_curr_accel_x, max_curr_accel_y, max_curr_accel_z;
    int dy_thres_accel_x = 0, dy_thres_accel_y = 0, dy_thres_accel_z = 0;
    int dy_chan_accel_x, dy_chan_accel_y, dy_chan_accel_z;
    int sample_new = 0, sample_old = 0;
    int step_size = 200, step_count = 0;
    int active_axis = 0, interval = 500000;
    int step_changed = 0;
    float distance_in_meter = 0.0; // Own
    float dis_per_step_in_meter = 0.762; // Own
///
/////////////////////| End of step counter |///////////////////////

///// Own Genaral Variables //////
#define AWAKE_BUFFER_SIZE 10
int16_t awake_buffer[AWAKE_BUFFER_SIZE];
int awake_buffer_index = 0;
bool isWatchAwake = false;

#define I2C1_NODE DT_NODELABEL(mpu6050)
static const struct i2c_dt_spec mpu_spec =  I2C_DT_SPEC_GET(I2C1_NODE);

// mpu6050 begin
bool mpu6050_begin(void){
  init_timeStamp();
  if(i2c_is_ready_dt(&mpu_spec)){ // Start of check i2c initilization
	    char array[1] = {NULL};
		  if(i2c_write_dt(&mpu_spec,array,sizeof(array))){
			    return false;
		  }
	    else{}
      }
  else{
	     return false;
	} // End of check i2c initilization
  k_msleep(1000);
  return mpu6050_init();
}

bool mpu6050_register_write(uint8_t register_address, uint8_t value)
{
    uint8_t tx_buf[MPU6050_ADDRESS_LEN+1];
	
    //Write the register address and data into transmit buffer
    tx_buf[0] = register_address;
    tx_buf[1] = value;

    //Wait until the transmission of the data is finished

    // if there is no error then return true else return false
    if (!(i2c_write_dt(&mpu_spec,tx_buf,sizeof(tx_buf))))
    {
        return true;
    }
    
    return false;	
}




/*
  A Function to read data from the MPU6050
*/ 
bool mpu6050_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    if (!(i2c_write_read_dt(&mpu_spec,&register_address,sizeof(register_address),destination,number_of_bytes)))
    {
        return true;
    }
    else{
      return false;
    }
}



/*
  A Function to verify the product id
  (its a basic test to check if we are communicating with the right slave, every type of I2C Device has 
  a special WHO_AM_I register which holds a specific value, we can read it from the MPU6050 or any device
  to confirm we are communicating with the right device)
*/ 
bool mpu6050_verify_product_id(void)
{
    uint8_t who_am_i; // create a variable to hold the who am i value


    // Note: All the register addresses including WHO_AM_I are declared in 
    // MPU6050.h file, you can check these addresses and values from the
    // datasheet of your slave device.
    if (mpu6050_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1))
    {
        //printk("[INFO : 1] who am i =  %u\r\n",who_am_i);
        //printk("[INFO : 2] who am i =  %u\r\n",ADDRESS_WHO_AM_I);
        if (who_am_i != MPU6050_WHO_AM_I)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}


/*
  Function to initialize the mpu6050
*/ 
bool mpu6050_init(void)
{   
  bool transfer_succeeded = true;
	
  //Check the id to confirm that we are communicating with the right device
  transfer_succeeded &= mpu6050_verify_product_id();
	
  if(mpu6050_verify_product_id() == false)
    {
	    return false;
    }

  // Set the registers with the required values, see the datasheet to get a good idea of these values
  (void)mpu6050_register_write(MPU_PWR_MGMT1_REG , 0x00); 
  (void)mpu6050_register_write(MPU_SAMPLE_RATE_REG , 0x07); 
  (void)mpu6050_register_write(MPU_CFG_REG , 0x06); 						
  (void)mpu6050_register_write(MPU_INT_EN_REG, 0x00); 
  (void)mpu6050_register_write(MPU_GYRO_CFG_REG , 0x18); 
  (void)mpu6050_register_write(MPU_ACCEL_CFG_REG,0x00);   		

  return transfer_succeeded;
}



/*
  Read the Gyro values from the MPU6050's internal Registers
*/ 
bool MPU6050_ReadGyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z )
{
  uint8_t buf[6]; 
  
  bool ret = false;	
	
  if(mpu6050_register_read(MPU6050_GYRO_OUT,  buf, 6) == true)
  {
    *pGYRO_X = (buf[0] << 8) | buf[1];
    if(*pGYRO_X & 0x8000) *pGYRO_X-=65536;
		
    *pGYRO_Y= (buf[2] << 8) | buf[3];
    if(*pGYRO_Y & 0x8000) *pGYRO_Y-=65536;
	
    *pGYRO_Z = (buf[4] << 8) | buf[5];
    if(*pGYRO_Z & 0x8000) *pGYRO_Z-=65536;
		
    ret = true;
	}

  return ret;
}	
/*
  A Function to read accelerometer's values from the internal registers of MPU6050
*/ 
bool MPU6050_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z )
{
  uint8_t buf[6];
  bool ret = false;		
  
  if(mpu6050_register_read(MPU6050_ACC_OUT, buf, 6) == true)
  {
    mpu6050_register_read(MPU6050_ACC_OUT, buf, 6);
    
    *pACC_X = (buf[0] << 8) | buf[1];
    if(*pACC_X & 0x8000) *pACC_X-=65536;

    *pACC_Y= (buf[2] << 8) | buf[3];
    if(*pACC_Y & 0x8000) *pACC_Y-=65536;

    *pACC_Z = (buf[4] << 8) | buf[5];
    if(*pACC_Z & 0x8000) *pACC_Z-=65536;
		
    ret = true;
    }
  
  return ret;
}
///// Own Function No. 1 //////
bool awakeCheck(int16_t zeeValue) {
    // Add the new zeeValue to the awake_buffer
    awake_buffer[awake_buffer_index] = zeeValue;
    awake_buffer_index = (awake_buffer_index + 1) % AWAKE_BUFFER_SIZE;
    // Calculate the average of the values in the awake_buffer
    int16_t sum = 0;
    for (int i = 0; i < AWAKE_BUFFER_SIZE; i++) {
        sum += awake_buffer[i];
    }
    int16_t average = sum / AWAKE_BUFFER_SIZE;
    // Check if zeeValue is above the threshold and the watch is not already awake
    if (zeeValue > 17000 && average <= 17000 && !isWatchAwake) {
        isWatchAwake = true;
        return true;
    }
    // Check if zeeValue falls below the threshold and the watch is already awake
    if (zeeValue <= 17000 && average <= 17000 && isWatchAwake) {
        isWatchAwake = false;
    }
    return false;
}
void esp32_i2c_read_bit
(
	uint8_t register_address,
	uint8_t bit_number,
	uint8_t* data
)
{
	uint8_t bit;
	//uint8_t count = esp32_i2c_read_byte(device_address, register_address, &bit);
  i2c_reg_read_byte_dt(&mpu_spec,register_address,&bit);

	*data = bit & (1 << bit_number);
}
bool mpu6050_get_int_dmp_status()
{
    esp32_i2c_read_bit
    (
        MPU6050_REGISTER_INT_STATUS,
        MPU6050_INTERRUPT_DMP_INT_BIT,
        step_buffer
    );

    return (step_buffer[0]);
}
void mpu6050_get_acceleration(mpu6050_acceleration_t* data)
{
  int16_t AccValue[3];
    if(MPU6050_ReadAcc(&AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
      {
        data->accel_x = AccValue[0];
        data->accel_y = AccValue[1];
        data->accel_z = AccValue[2];
        //printk("[INFO] X %u | Y %u | Z %u\r\n",AccValue[0],AccValue[1],AccValue[2]);
      }//if(MPU6050_ReadAcc(&AccValue[0],
      else
      {
        printk("\033[31mReading ACC values Failed!!!\033[0m\r\n"); // if reading was unsuccessful then let the user know about it
      }
    
}
void esp32_i2c_read_bits
(
	uint8_t register_address,
	uint8_t bit_start,
	uint8_t size,
	uint8_t* data
)
{
	uint8_t bit;
  if (!(i2c_reg_read_byte_dt(&mpu_spec, register_address, &bit))) {
		uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);

		bit &= mask;
		bit >>= (bit_start - size + 1);
		*data = bit;
	}
}
uint8_t mpu6050_get_full_scale_accel_range()
{
    esp32_i2c_read_bits
    (
        MPU6050_REGISTER_ACCEL_CONFIG,
        MPU6050_ACONFIG_AFS_SEL_BIT,
        MPU6050_ACONFIG_AFS_SEL_LENGTH,
        step_buffer
    );

    return (step_buffer[0]);
}
float mpu6050_get_accel_res(uint8_t accel_scale)
{
    float accel_res = 0;

    switch (accel_scale) {
        case 0:
            accel_res = (float)2.0 / 32768.0;
            break;
        case 1:
            accel_res = (float)4.0 / 32768.0;
            break;
        case 2:
            accel_res = (float)8.0 / 32768.0;
            break;
        case 3:
            accel_res = (float)16.0 / 32768.0;
            break;
    }

    return (accel_res);
}
void mpu6050_get_rotation(mpu6050_rotation_t* data)
{
    int16_t GyroValue[3];
    if(MPU6050_ReadGyro(&GyroValue[0], &GyroValue[1], &GyroValue[2]) == true) // read the gyro values from mpu6050's internal registers and save them in another array
      {
        data->gyro_x = GyroValue[0];
        data->gyro_y = GyroValue[1];
        data->gyro_z = GyroValue[2];
      }

      else
      {
        printk("\033[31mReading GYRO values Failed!!!\033[0m\r\n");
      }
}
uint8_t mpu6050_get_full_scale_gyro_range()
{
    esp32_i2c_read_bits
    (
        MPU6050_REGISTER_GYRO_CONFIG,
        MPU6050_GCONFIG_FS_SEL_BIT,
        MPU6050_GCONFIG_FS_SEL_LENGTH,
        step_buffer
    );

    return (step_buffer[0]);
}
float mpu6050_get_gyro_res(uint8_t gyro_scale)
{
    float gyro_res = 0;
    
    switch (gyro_scale) {
        case 0:
            gyro_res = (float)250.0 / 32768.0;
            break;
        case 1:
            gyro_res = (float)500.0 / 32768.0;
            break;
        case 2:
            gyro_res = (float)1000.0 / 32768.0;
            break;
        case 3:
            gyro_res = (float)2000.0 / 32768.0;
            break;
    }

    return (gyro_res);
}
void mpu6050_madgwick_quaternion_update
(
    float accel_x,
    float accel_y,
    float accel_z,
    float gyro_x,
    float gyro_y,
    float gyro_z
)
{
    float func_1, func_2, func_3;
    float j_11o24, j_12o23, j_13o22, j_14o21, j_32, j_33;
    float q_dot_1, q_dot_2, q_dot_3, q_dot_4;
    float hat_dot_1, hat_dot_2, hat_dot_3, hat_dot_4;
    float gyro_x_err, gyro_y_err, gyro_z_err;
    float gyro_x_bias, gyro_y_bias, gyro_z_bias;
    float norm;
 
    float half_q1 = 0.5f * quart[0];
    float half_q2 = 0.5f * quart[1];
    float half_q3 = 0.5f * quart[2];
    float half_q4 = 0.5f * quart[3];
    float double_q1 = 2.0f * quart[0];
    float double_q2 = 2.0f * quart[1];
    float double_q3 = 2.0f * quart[2];
    float double_q4 = 2.0f * quart[3];
 
    // Normalise accelerometer measurement:
    norm = sqrt((int64_t)(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z));
    
    // Handle NaN:
    if (norm == 0.0f)
        return;
    
    norm = (float)1.0f / norm;
    accel_x *= norm;
    accel_y *= norm;
    accel_z *= norm;
    
    // Compute the objective function and Jacobian:
    func_1 = double_q2 * quart[3] - double_q1 * quart[2] - accel_x;
    func_2 = double_q1 * quart[1] + double_q3 * quart[3] - accel_y;
    func_3 = 1.0f - double_q2 * quart[3] - double_q3 * quart[2] - accel_z;
    j_11o24 = double_q3;
    j_12o23 = double_q4;
    j_13o22 = double_q1;
    j_14o21 = double_q2;
    j_32 = 2.0f * j_14o21;
    j_33 = 2.0f * j_11o24;
          
    // Compute the gradient (matrix multiplication):
    hat_dot_1 = j_14o21 * func_2 - j_11o24 * func_1;
    hat_dot_2 = j_12o23 * func_1 + j_13o22 * func_2 - j_32 * func_3;
    hat_dot_3 = j_12o23 * func_2 - j_33 *func_3 - j_13o22 * func_1;
    hat_dot_4 = j_14o21 * func_1 + j_11o24 * func_2;
            
    // Normalize the gradient:
    norm = sqrt((int64_t)(hat_dot_1 * hat_dot_1 + hat_dot_2 * hat_dot_2 + hat_dot_3 * hat_dot_3 + hat_dot_4 * hat_dot_4));
    hat_dot_1 = (float)hat_dot_1 / norm;
    hat_dot_2 = (float)hat_dot_2 / norm;
    hat_dot_3 = (float)hat_dot_3 / norm;
    hat_dot_4 = (float)hat_dot_4 / norm;
            
    // Compute estimated gyroscope biases:
    gyro_x_err = double_q1 * hat_dot_2 - double_q2 * hat_dot_1 - double_q3 * hat_dot_4 + double_q4 * hat_dot_3;
    gyro_y_err = double_q1 * hat_dot_3 + double_q2 * hat_dot_4 - double_q3 * hat_dot_1 - double_q4 * hat_dot_2;
    gyro_z_err = double_q1 * hat_dot_4 - double_q2 * hat_dot_3 + double_q3 * hat_dot_2 - double_q4 * hat_dot_1;
            
    // Compute and remove gyroscope biases:
    gyro_x_bias += gyro_x_err * delta_t * ZETA;
    gyro_y_bias += gyro_y_err * delta_t * ZETA;
    gyro_z_bias += gyro_z_err * delta_t * ZETA;
            
    // Compute the quaternion derivative:
    q_dot_1 = -half_q2 * gyro_x - half_q3 * gyro_y - half_q4 * gyro_z;
    q_dot_2 =  half_q1 * gyro_x + half_q3 * gyro_z - half_q4 * gyro_y;
    q_dot_3 =  half_q1 * gyro_y - half_q2 * gyro_z + half_q4 * gyro_x;
    q_dot_4 =  half_q1 * gyro_z + half_q2 * gyro_y - half_q3 * gyro_x;
 
    // Compute then integrate estimated quaternion derivative:
    quart[0] += (q_dot_1 -(BETA * hat_dot_1)) * delta_t;
    quart[1] += (q_dot_2 -(BETA * hat_dot_2)) * delta_t;
    quart[2] += (q_dot_3 -(BETA * hat_dot_3)) * delta_t;
    quart[3] += (q_dot_4 -(BETA * hat_dot_4)) * delta_t;
 
    // Normalize the quaternion:
    norm = sqrt(quart[0] * quart[0] + quart[1] * quart[1] + quart[2] * quart[2] + quart[3] * quart[3]);
    norm = 1.0f / norm;
    quart[0] *= norm;
    quart[1] *= norm;
    quart[2] *= norm;
    quart[3] *= norm;
}
const char* mpu6050_get_tag()
{
    return (TAG_MPU6050);
}
void step_counter(void){
  while(step_counter_start){
    if (!mpu6050_get_int_dmp_status()) {
        mpu6050_get_acceleration(&accel);

        range = mpu6050_get_full_scale_accel_range();
        accel_res = (float)mpu6050_get_accel_res(range);
        if(awakeCheck(accel.accel_z)){
          printk("Smart Watch Is Waken Up !!!!");
          k_msleep(3000);
        }
        accel_g_x = (float) accel.accel_x * accel_res - accel_bias[0];
        accel_g_y = (float) accel.accel_y * accel_res - accel_bias[1];
        accel_g_z = (float) accel.accel_z * accel_res - accel_bias[2];

        if (accel_g_x < 0)
            accel_g_x *= -1;
        if (accel_g_y < 0)
            accel_g_y *= -1;
        if (accel_g_z < 0)
            accel_g_z *= -1;

        mpu6050_get_rotation(&gyro);

        range = mpu6050_get_full_scale_gyro_range();
        gyro_res = (float)mpu6050_get_gyro_res(range);

        gyro_ds_x = (float) gyro.gyro_x * gyro_res - gyro_bias[0];
        gyro_ds_y = (float) gyro.gyro_y * gyro_res - gyro_bias[1];
        gyro_ds_z = (float) gyro.gyro_z * gyro_res - gyro_bias[2];

        mpu6050_madgwick_quaternion_update
        (
            accel_g_x,
            accel_g_y,
            accel_g_z,
            gyro_ds_x * (float)(PI / 180.0f),
            gyro_ds_y * (float)(PI / 180.0f),
            gyro_ds_z * (float)(PI / 180.0f)
        );

        accel_int_x = 1000 * accel_g_x;
        accel_int_y = 1000 * accel_g_y;
        accel_int_z = 1000 * accel_g_z;

        accel_x_avg_buff[accel_x_avg_buff_count] = accel_int_x;
        accel_x_avg_buff_count++;
        accel_x_avg_buff_count %= AVG_BUFF_SIZE;
        accel_x_avg = 0;

        for (int i = 0; i < AVG_BUFF_SIZE; i++)
            accel_x_avg += accel_x_avg_buff[i];
            
        accel_x_avg = (float)(accel_x_avg / AVG_BUFF_SIZE);

        accel_y_avg_buff[accel_y_avg_buff_count] = accel_int_y;
        accel_y_avg_buff_count++;
        accel_y_avg_buff_count %= AVG_BUFF_SIZE;
        accel_y_avg = 0;

        for (int i = 0; i < AVG_BUFF_SIZE; i++)
            accel_y_avg += accel_y_avg_buff[i];
            
        accel_y_avg = (float)accel_y_avg / AVG_BUFF_SIZE;

        accel_z_avg_buff[accel_z_avg_buff_count] = accel_int_z;
        accel_z_avg_buff_count++;
        accel_z_avg_buff_count %= AVG_BUFF_SIZE;
        accel_z_avg = 0;

        for (int i = 0; i < AVG_BUFF_SIZE; i++)
            accel_z_avg += accel_z_avg_buff[i];
            
        accel_z_avg = (float)accel_z_avg / AVG_BUFF_SIZE;

        now_time = get_uTimeStamp();

        if (now_time - prev_time >= interval) {
            prev_time = now_time;
            
            min_curr_accel_x = min_reg_accel_x;
            max_curr_accel_x = max_reg_accel_x;
            dy_thres_accel_x = (float)((min_curr_accel_x + max_curr_accel_x) / 2);
            dy_chan_accel_x = (max_curr_accel_x - min_curr_accel_x);
            min_reg_accel_x = accel_x_avg;
            max_reg_accel_x = accel_x_avg;
            min_curr_accel_y = min_reg_accel_y;
            max_curr_accel_y = max_reg_accel_y;
            dy_thres_accel_y = (float)((min_curr_accel_y + max_curr_accel_y) / 2);
            dy_chan_accel_y = (max_curr_accel_y - min_curr_accel_y);
            min_reg_accel_y = accel_y_avg;
            max_reg_accel_y = accel_y_avg;
            min_curr_accel_z = min_reg_accel_z;
            max_curr_accel_z = max_reg_accel_z;
            dy_thres_accel_z = (float)((min_curr_accel_z + max_curr_accel_z) / 2);
            dy_chan_accel_z = (max_curr_accel_z - min_curr_accel_z);
            min_reg_accel_z = accel_z_avg;
            max_reg_accel_z = accel_z_avg;
                
            if (dy_chan_accel_x >= dy_chan_accel_y &&
                dy_chan_accel_x >= dy_chan_accel_z) {
                if (active_axis != 0) {
                    sample_old = 0;
                    sample_new = accel_x_avg;
                }
                active_axis = 0;
            } else if (dy_chan_accel_y >= dy_chan_accel_x &&
                dy_chan_accel_y >= dy_chan_accel_z) {
                    if (active_axis != 1) {
                    sample_old = 0;
                    sample_new = accel_y_avg;
                }
                active_axis = 1;
            } else {
                if (active_axis != 2) {
                    sample_old = 0;
                    sample_new = accel_z_avg;
                }
                active_axis = 2;
            }

        } else if (now_time < 500) {
            if (min_reg_accel_x > accel_x_avg)
                min_reg_accel_x = accel_x_avg;
            if (max_reg_accel_x < accel_x_avg)
                max_reg_accel_x = accel_x_avg;
            if (min_reg_accel_y > accel_y_avg)
                min_reg_accel_y = accel_y_avg;
            if (max_reg_accel_y < accel_y_avg)
                max_reg_accel_y = accel_y_avg;
            if (min_reg_accel_z > accel_z_avg)
                min_reg_accel_z = accel_z_avg;
            if (max_reg_accel_z < accel_z_avg)
                max_reg_accel_z = accel_z_avg;
        }
            
        sample_old = sample_new;
        switch (active_axis) {
            case 0:
                if (accel_x_avg - sample_old > step_size ||
                    accel_x_avg - sample_old < -step_size) {
                    sample_new = accel_x_avg;
                    if (sample_old > dy_thres_accel_x &&
                        sample_new < dy_thres_accel_x) {
                        step_count++;
                        update_calories(step_count);
                        distance_in_meter = (float)step_count * dis_per_step_in_meter; //Own // Step * per step distance = distance
                        step_changed = 1;
                    }
                }
                break;
            case 1:
                if (accel_y_avg - sample_old > step_size ||
                    accel_y_avg - sample_old < -step_size) {
                    sample_new = accel_y_avg;
                    if (sample_old > dy_thres_accel_y &&
                        sample_new < dy_thres_accel_y) {
                        step_count++;
                        update_calories(step_count);
                        distance_in_meter = (float)step_count * dis_per_step_in_meter; //Own // Step * per step distance = distance
                        step_changed = 1;
                    }
                }
                break;
            case 2:
                if (accel_z_avg - sample_old > step_size ||
                    accel_z_avg - sample_old < -step_size) {
                    sample_new = accel_z_avg;
                    if (sample_old > dy_thres_accel_z &&
                        sample_new < dy_thres_accel_z) {
                        step_count++;
                        update_calories(step_count);
                        distance_in_meter = (float)step_count * dis_per_step_in_meter; //Own // Step * per step distance = distance
                        step_changed = 1;
                    }
                }
                break;
        }

        if (step_changed) {
            printk("Tage : %s | Steps : %d | Distance : %f meter | Calories : %f cal\r\n",mpu6050_get_tag(), step_count, distance_in_meter,calories);
            step_changed = 0;
        }
        k_msleep(10);
     }
   }
}
void init_mpu6050(void){
  while(!(mpu6050_begin())) // wait until MPU6050 sensor is successfully initialized
    {
      printk("\033[31mFaild MPU_6050 initialization!!!\033[0m\r\n");
      k_msleep(1000);
    }

    printk("\033[32mMPU6050 Init Successfully!!!\033[0m\r\n"); // If init Successfully
    printk("\033[32mReading Values from MPU6050\033[0m\r\n"); // display a message to let the user know that the device is starting to read the values
    k_msleep(2000);
}

void update_calories(int steps){
    float _weight = 85; // Weight in Kilograms
    float _height = 1.706; // Height in Meters
    float _speed = 1.34; // Speed in m/s for average running
    float MET = 3.5; // MET = 3.0 - 4.0 (For running)... MET for Metabolic equivalents
    //// Get time
    float _stride = _height * 0.414; // This is constant
    float _distance = _stride * steps; // Measure distance
    float _time = (float)((float)_distance/_speed)/60; // get time
    ////
    calories = (float)(_time * MET * 3.5 * _weight)/200;
}


