#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <string.h>
#include<math.h>
#include <stdbool.h>
#include "max30102.h"
#include "hr_spo2_algo.h"
#include "TimeStamp.h"


///////////////| General parameters definations |//////////////
#define Minimum_Threshold_hr  60 //Lowest heart rate is 60 after human death
#define Maximum_Threshold_hr 200 // Highest hear rate is 200 after human death
#define Lowest_spo2 90 // Lowest spo2 after human death
// 45000 to 75000 IR detected
/////////////////////////////
uint32_t cc; // for test unit var values // for test unit var values
uint8_t powerLevel = 0x1F;
uint8_t sampleAverage = 4;
uint8_t ledMode = 3;
int sampleRate = 400;
int pulseWidth = 411;
int adcRange = 4096;
uint8_t transmitting=0;
///////////// This from private in cpp class//////////
uint8_t revisionID; 
//uint8_t _i2caddr = MAX30105_ADDRESS;
uint8_t _i2caddr = 0x57;

//activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
uint8_t activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
//i2c initialize
#define MAX3010X_NODE DT_NODELABEL(max3010x)
static const struct i2c_dt_spec max3010x_spec = I2C_DT_SPEC_GET(MAX3010X_NODE);
uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

//////| Struct defined in header in class |///////
#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
  typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint32_t green[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
  } sense_struct; //This is our circular buffer of readings from the sensor
sense_struct sense;
///////////////| Others variables|/////////////////////
uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

// Status Registers
//static const uint8_t MAX30105_INTSTAT1 =		0x00;
//static const uint8_t MAX30105_INTSTAT2 =		0x01;
//static const uint8_t MAX30105_INTENABLE1 =		0x02;
//static const uint8_t MAX30105_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30105_FIFOREADPTR = 	0x06;
static const uint8_t MAX30105_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 		0x08;
static const uint8_t MAX30105_MODECONFIG = 		0x09;
static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
//static const uint8_t MAX30105_DIETEMPINT = 		0x1F;
//static const uint8_t MAX30105_DIETEMPFRAC = 	0x20;
//static const uint8_t MAX30105_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
//static const uint8_t MAX30105_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 		0xFE;
static const uint8_t MAX30105_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
//static const uint8_t MAX30105_INT_A_FULL_MASK =		(uint8_t)~0b10000000;
//static const uint8_t MAX30105_INT_A_FULL_ENABLE = 	0x80;
//static const uint8_t MAX30105_INT_A_FULL_DISABLE = 	0x00;

//static const uint8_t MAX30105_INT_DATA_RDY_MASK = (uint8_t)~0b01000000;
//static const uint8_t MAX30105_INT_DATA_RDY_ENABLE =	0x40;
//static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

//static const uint8_t MAX30105_INT_ALC_OVF_MASK = (uint8_t)~0b00100000;
//static const uint8_t MAX30105_INT_ALC_OVF_ENABLE = 	0x20;
//static const uint8_t MAX30105_INT_ALC_OVF_DISABLE = 0x00;

//static const uint8_t MAX30105_INT_PROX_INT_MASK = (uint8_t)~0b00010000;
//static const uint8_t MAX30105_INT_PROX_INT_ENABLE = 0x10;
//static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

//static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK = (uint8_t)~0b00000010;
//static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
//static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK =	(uint8_t)~0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10;
//static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

//static const uint8_t MAX30105_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
//static const uint8_t MAX30105_SHUTDOWN_MASK = 	0x7F;
//static const uint8_t MAX30105_SHUTDOWN = 		0x80;
//static const uint8_t MAX30105_WAKEUP = 			0x00;

static const uint8_t MAX30105_RESET_MASK = 		0xBF;
static const uint8_t MAX30105_RESET = 			0x40;

static const uint8_t MAX30105_MODE_MASK = 		0xF8;
static const uint8_t MAX30105_MODE_REDONLY = 	0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30105_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30105_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30105_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30105_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30105_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30105_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30105_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK = 0xF8;
static const uint8_t MAX30105_SLOT2_MASK = 0x8F;
static const uint8_t MAX30105_SLOT3_MASK = 0xF8;
static const uint8_t MAX30105_SLOT4_MASK = 0x8F;

//static const uint8_t SLOT_NONE = 0x00;
static const uint8_t SLOT_RED_LED = 0x01;
static const uint8_t SLOT_IR_LED = 0x02;
static const uint8_t SLOT_GREEN_LED = 0x03;
//static const uint8_t SLOT_NONE_PILOT = 	0x04;
//static const uint8_t SLOT_RED_PILOT =  0x05;
//static const uint8_t SLOT_IR_PILOT = 0x06;
//static const uint8_t SLOT_GREEN_PILOT = 0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;

uint32_t aun_ir_buffer[100]; //infrared LED sensor data
uint32_t aun_red_buffer[100];  //red LED sensor data
int32_t n_ir_buffer_length = 100; //data length
int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  // 1 if the calculated SpO2 value is valid else 0
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  // 1 if the calculated heart rate value is valid else 0
uint8_t uch_dummy;


 ////////////////////////////////| Initialize Main Functions |/////////////////////////////////////////////////
bool max30102_Begin(){
     init_timeStamp();
     if(i2c_is_ready_dt(&max3010x_spec)){ // Start of check i2c initilization
	       char array[1] = {NULL};
		     if(i2c_write_dt(&max3010x_spec,array,sizeof(array))){
			       return false;
		     }
	       else{}
     }
     else{
	       return false;
	   } // End of check i2c initilization
     if(readPartID() != MAX_30105_EXPECTEDPARTID){
        return false;
     }
    
     readRevisionID();
     max30102_Setup();
     return true;
}

////////////////////////////
void max30102_Setup(){
    softReset(); //Reset all configuration, threshold, and data registers to POR values


    //FIFO Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    //The chip will average multiple samples of same type together if you wish
    if (sampleAverage == 1) setFIFOAverage(MAX30105_SAMPLEAVG_1); //No averaging per FIFO record
    else if (sampleAverage == 2) setFIFOAverage(MAX30105_SAMPLEAVG_2);
    else if (sampleAverage == 4) setFIFOAverage(MAX30105_SAMPLEAVG_4);
    else if (sampleAverage == 8) setFIFOAverage(MAX30105_SAMPLEAVG_8);
    else if (sampleAverage == 16) setFIFOAverage(MAX30105_SAMPLEAVG_16);
    else if (sampleAverage == 32) setFIFOAverage(MAX30105_SAMPLEAVG_32);
    else setFIFOAverage(MAX30105_SAMPLEAVG_4);
    //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
    enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) setLEDMode(MAX30105_MODE_MULTILED); //Watch all three LED channels
  else if (ledMode == 2) setLEDMode(MAX30105_MODE_REDIRONLY); //Red and IR
  else setLEDMode(MAX30105_MODE_REDONLY); //Red only
  activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) setADCRange(MAX30105_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) setADCRange(MAX30105_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) setADCRange(MAX30105_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) setADCRange(MAX30105_ADCRANGE_16384); //62.5pA per LSB
  else setADCRange(MAX30105_ADCRANGE_2048);

  if (sampleRate < 100) setSampleRate(MAX30105_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) setSampleRate(MAX30105_SAMPLERATE_100);
  else if (sampleRate < 400) setSampleRate(MAX30105_SAMPLERATE_200);
  else if (sampleRate < 800) setSampleRate(MAX30105_SAMPLERATE_400);
  else if (sampleRate < 1000) setSampleRate(MAX30105_SAMPLERATE_800);
  else if (sampleRate < 1600) setSampleRate(MAX30105_SAMPLERATE_1000);
  else if (sampleRate < 3200) setSampleRate(MAX30105_SAMPLERATE_1600);
  else if (sampleRate == 3200) setSampleRate(MAX30105_SAMPLERATE_3200);
  else setSampleRate(MAX30105_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) setPulseWidth(MAX30105_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215) setPulseWidth(MAX30105_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) setPulseWidth(MAX30105_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) setPulseWidth(MAX30105_PULSEWIDTH_411); //18 bit resolution
  else setPulseWidth(MAX30105_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  setPulseAmplitudeGreen(powerLevel);
  setPulseAmplitudeProximity(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
  if (ledMode > 2) enableSlot(3, SLOT_GREEN_LED);
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}
////////////////////////////
uint32_t max30102_GetRed(void){
  //Check the sensor for new data for 250ms
  if(safeCheck(250)){
    return (sense.red[sense.head]);
    }
  else{
    return(0); //Sensor failed to find new data
    }
}
uint32_t max30102_GetIR(void){
    //Check the sensor for new data for 250ms
  if(safeCheck(250)){
    
    return (sense.IR[sense.head]);
    }
  else{

    return(0); //Sensor failed to find new data
    }
}
uint32_t max30102_GetGreen(void){
    //Check the sensor for new data for 250ms
  if(safeCheck(250)){
    
    return (sense.green[sense.head]);
    }
  else{
    return(0); //Sensor failed to find new data
    }
}


/////////////////////////////////| Initialize Nested Functions |/////////////////////////////////////////////////
///////| (2) |///////////
void writeRegister8(uint8_t reg, uint8_t value1)
{
  uint8_t buff[2] = {reg,value1};
  i2c_write_dt(&max3010x_spec,buff,sizeof(buff));
}
///////| (3) |///////////

uint8_t readRegister8(uint8_t reg)
{
  uint8_t quantity = 1;
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  i2c_write_read_dt(&max3010x_spec,&reg,sizeof(reg),rxBuffer,1);
  
  rxBufferIndex = 0;
  rxBufferLength = sizeof(rxBuffer);
  if(available())
  {
    
    return (read());
  }
  
  return(0); //Fail
}

///////| (4) |///////////
uint8_t readPartID(){
    
    return readRegister8(MAX30105_PARTID);
}
///////| (5) |///////////
void readRevisionID() {
  revisionID = readRegister8(MAX30105_REVISIONID);
  
}
///////| (6) |///////////
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(reg, originalContents | thing);
}
///////| (7) |///////////

void softReset(void) {
  bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  uint32_t startTime =  get_mTimeStamp();

  while (((get_mTimeStamp()) - startTime) - startTime < 100)
  {
    uint8_t response = readRegister8(MAX30105_MODECONFIG);
    if ((response & MAX30105_RESET) == 0) break; //We're done!
    k_msleep(1); //Let's not over burden the I2C bus
  }
}
///////| (8) |///////////
void setFIFOAverage(uint8_t numberOfSamples) {
  
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
  
}
///////| (9) |///////////
void enableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}
///////| (10) |///////////
void setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
  
}
///////| (11) |///////////
void setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
  
}
///////| (12) |///////////
void setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}
///////| (13) |///////////
void setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}
///////| (14) |///////////
void setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(MAX30105_LED1_PULSEAMP, amplitude);
  
}
///////| (15) |///////////
void setPulseAmplitudeIR(uint8_t amplitude){
  writeRegister8(MAX30105_LED2_PULSEAMP, amplitude);
}
///////| (16) |///////////
void setPulseAmplitudeGreen(uint8_t amplitude){
  writeRegister8(MAX30105_LED3_PULSEAMP, amplitude);
}
///////| (17) |///////////
void setPulseAmplitudeProximity(uint8_t amplitude){
  writeRegister8(MAX30105_LED_PROX_AMP, amplitude);
}
///////| (18) |///////////
void enableSlot(uint8_t slotNumber, uint8_t device) {

  //uint8_t originalContents;
  
  switch (slotNumber) {
    case (1):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}
///////| (19) |///////////
void clearFIFO(void) {
  writeRegister8(MAX30105_FIFOWRITEPTR, 0);
  writeRegister8(MAX30105_FIFOOVERFLOW, 0);
  writeRegister8(MAX30105_FIFOREADPTR, 0);
}
///////| (20) |///////////
bool safeCheck(uint32_t maxTimeToCheck)
{
  uint32_t markTime = get_mTimeStamp();
  while(1)
  {     
        if(((get_mTimeStamp()) - markTime) > maxTimeToCheck){

          return false;
          
        }
	if(check() == true){ //We found new data!
         //maybe need to test the loop
          
	  return(true);
          }
        else{
        //maybe need to test the loop
        
        return(false);
        }
        //k_msleep(1);
  }
}
///////| (21) |///////////
bool check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  uint8_t readPointer = getReadPointer();
  uint8_t writePointer = getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    i2c_write_dt(&max3010x_spec,&MAX30105_FIFODATA,sizeof(MAX30105_FIFODATA));
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      requestFrom(toGet);
      
      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;
        
        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = read();
        temp[1] = read();
        temp[0] = read();

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));
		
		tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = read();
          temp[1] = read();
          temp[0] = read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits
          
		  sense.IR[sense.head] = tempLong;
        }

        if (activeLEDs > 2)
        {
          //Burst read three more bytes - Green
          temp[3] = 0;
          temp[2] = read();
          temp[1] = read();
          temp[0] = read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.green[sense.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr
  
  return (true);//(numberOfSamples); //Let the world know how much new data we found
}
///////| (22) |///////////
//Read the FIFO Read Pointer
uint8_t getReadPointer(void) {
  
  return (readRegister8(MAX30105_FIFOREADPTR));
}
///////| (23) |///////////
//Read the FIFO Write Pointer
uint8_t getWritePointer(void) {

  return (readRegister8(MAX30105_FIFOWRITEPTR));
}
///////| (24) |///////////
uint8_t requestFrom(uint8_t quantity)
{
  
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
    
  }
   i2c_read_dt(&max3010x_spec,rxBuffer,quantity);
   
  //APP_ERROR_CHECK(err_code);
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = sizeof(rxBuffer);
  return rxBufferLength;
}
///////| (25) |///////////
int read(void)
{
  int value = -1;
  
  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }
  
  return value;
}
///////| (26) |///////////
void beginTransmission(void){

  // indicate that we are transmitting
  transmitting = 1;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
 
}
///////| (27) |///////////
void write(uint8_t data){
  
  if(transmitting){
  // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer   
    txBufferLength = txBufferIndex;
    
  }
  else{
  
  // in slave send mode
    // reply to master
    i2c_write_dt(&max3010x_spec,&data,sizeof(data));
  }
}
///////| (28) |///////////
uint32_t endTransmission(void)
{
  uint32_t err_code_write = i2c_write_dt(&max3010x_spec,txBuffer,sizeof(txBuffer));
  
  if(err_code_write == 0) {
        err_code_write=255;
  }
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return err_code_write;
}
///////| (29) |///////////
//Tell caller how many samples are available
uint8_t available(void)
{
  
  return rxBufferLength - rxBufferIndex;

}
void init_heart_rate(void){
  int i = 0;
    for(i = 0; i < n_ir_buffer_length;i++)
    {
        k_msleep(10);
        aun_red_buffer[i] = max30102_GetRed();
        aun_ir_buffer[i] = max30102_GetIR();
    }
    //calc. from algorithm
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
}
///////| (31) |///////////
bool get_hr_spo2(uint32_t* p_hr, uint32_t* p_spo2){
  bool hr_ack = false;
  bool spo2_ack = false;
  init_heart_rate();
  int i = 25;
  for(i = 25; i < 100;i++){
    aun_ir_buffer[i - 25] = aun_ir_buffer[i];
    aun_red_buffer[i - 25] = aun_red_buffer [i];
  } // End of for(i = 25; i < 100;i++){
  //add the latest 25 samples
  i = 75;
  for(i = 75; i < 100; i++){
    k_msleep(10);
    aun_red_buffer[i] = max30102_GetRed();
    aun_ir_buffer[i] = max30102_GetIR();
  } // End of for(i = 75; i < 100; i++){
  //calc. hr and spo2
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
  if(ch_hr_valid){
    if(n_heart_rate<Minimum_Threshold_hr){
      n_heart_rate = Minimum_Threshold_hr;
      *p_hr = n_heart_rate;
      hr_ack = true;
    } // End of if(n_heart_rate<Minimum_Threshold_hr){
    else if(n_heart_rate>Maximum_Threshold_hr){
      n_heart_rate = Maximum_Threshold_hr;
      *p_hr = n_heart_rate;
      hr_ack = true;
    } // End of else if(n_heart_rate>Maximum_Threshold_hr){
    else{
      *p_hr = n_heart_rate;
      hr_ack = true;
    } // End of else{
  } // End of if(ch_hr_valid){
  if(ch_spo2_valid){
    if(n_spo2<Lowest_spo2){
      n_spo2 = Lowest_spo2;
      *p_spo2 = n_spo2;
      spo2_ack = true;
    }// End of if(n_spo2<Lowest_spo2){
    else{
      *p_spo2 = n_spo2;
      spo2_ack = true;
    }// End of else{
  } // End of if(ch_spo2_valid){
  if(hr_ack && spo2_ack){
    return true;
  } // End of if(hr_ack && spo2_ack){
  else{
    return false;
  } // End of else{
} // End of bool get_hr_spo2(uint32_t* p_hr, uint32_t* p_spo2){
///////| (32) |///////////
void get_stress_level(char *p_stress){
  int newBaseLine=0;
  bool stress;
  uint32_t hrSamples[10];
  uint32_t sum=0;
  float lowThreshold=0.0,highThreshold=0.0;
  uint32_t currentHR,hr,spo2;
  //init_heart_rate();
  for(uint8_t i=0;i<10;i++){
    hrSamples[i]=0;
  } // End of for(uint8_t i=0;i<10;i++) to reset hrSamples buffer
  //hr = get_heart_rate();
  get_hr_spo2(&hr,&spo2);

  if(hr>Minimum_Threshold_hr){ // check condition
   for(uint8_t i=0;i<10;i++){
    //hr = get_heart_rate();
    get_hr_spo2(&hr,&spo2);
    if(hr>Minimum_Threshold_hr){ // check condition
     hrSamples[i] = hr;
    }// End of if(hr>Minimum_Threshold_hr) condition
    sum += hrSamples[i];
   } // End of for(uint8_t i=0;i<10;i++) condition
} // End of if(hr>Minimum_Threshold_hr) condition 

  newBaseLine = (int)sum/10;
  lowThreshold =  (float)newBaseLine * 0.8;
  highThreshold = (float)newBaseLine * 1.2;
  //currentHR = get_heart_rate();
  get_hr_spo2(&currentHR,&spo2);
  if(currentHR < lowThreshold){ // check condition if have stree
    stress = true;
    char status[60]= "\x1b[32m[INFO] Stress is Low\x1b[0m\0" ;
    memcpy(p_stress,status,sizeof(status));
    sum=0; // to reset value
    newBaseLine=0; // to reset value
    lowThreshold=0.0; // to reset value
    highThreshold=0.0; // to reset value
    currentHR=0; // to reset value
    //setStatus(1);
  } // End of if(currentHR < lowThreshold) condition

  else if(currentHR > highThreshold){ // check condition if have stress
    stress = true;
    char status[60]= "\x1b[32m[INFO] Stress is High\x1b[0m\0" ;
    memcpy(p_stress,status,sizeof(status));
    sum=0; // to reset value
    newBaseLine=0; // to reset value
    lowThreshold=0.0; // to reset value
    highThreshold=0.0; // to reset value
    currentHR=0; // to reset value
  } // End of else if(currentHR > highThreshold) condition

  else if(currentHR > lowThreshold && currentHR < highThreshold){ // check condition if no stress
    stress = false;
    char status[60]= "\x1b[32m[INFO] Stress is Moderate\x1b[0m\0" ;
    memcpy(p_stress,status,sizeof(status));
    sum=0; // to reset value
    newBaseLine=0; // to reset value
    lowThreshold=0.0; // to reset value
    highThreshold=0.0; // to reset value
    currentHR=0; // to reset value
  } // End of else
  else {
    char status[60]= "\x1b[33m[INFO] For stress place your finger\x1b[0m\0" ;
    memcpy(p_stress,status,sizeof(status));
  }
} // End of get_StresssLevel function