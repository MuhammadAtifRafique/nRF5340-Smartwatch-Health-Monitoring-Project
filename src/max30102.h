#ifndef MAX30102_H_
#define MAX30102_H_

//////////////////////////////////////////// | Header File | ///////////////////////////////////////////////
#include<stdbool.h>
#include<stdint.h>
#include<stdio.h>

#define MAX30105_ADDRESS    0x57 //7-bit I2C Address
#define I2C_BUFFER_LENGTH 32
#define BUFFER_LENGTH 32
#define HR_Times 25
//////////////////////| Main Functions |////////////////////////
bool max30102_Begin();
void max30102_Setup();
uint32_t max30102_GetRed(void);
uint32_t max30102_GetIR(void);
uint32_t max30102_GetGreen(void);
/////////////////////| Nested Functions |//////////////////////////
void writeRegister8(uint8_t reg, uint8_t value1);
uint8_t readRegister8(uint8_t reg);
uint8_t readPartID();
void readRevisionID();
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
void softReset(void);
void setFIFOAverage(uint8_t numberOfSamples);
void enableFIFORollover(void);
void setLEDMode(uint8_t mode);
void setADCRange(uint8_t adcRange);
void setSampleRate(uint8_t sampleRate);
void setPulseWidth(uint8_t pulseWidth);
void setPulseAmplitudeRed(uint8_t amplitude);
void setPulseAmplitudeIR(uint8_t amplitude);
void setPulseAmplitudeGreen(uint8_t amplitude);
void setPulseAmplitudeProximity(uint8_t amplitude);
void enableSlot(uint8_t slotNumber, uint8_t device);
void clearFIFO(void);
bool safeCheck(uint32_t maxTimeToCheck);
bool check(void);
uint8_t getReadPointer(void);
uint8_t getReadPointer(void);
uint8_t getWritePointer(void);
uint8_t requestFrom(uint8_t quantity);
int read(void);
void beginTransmission(void);
void write(uint8_t data);
uint32_t endTransmission(void);
uint8_t available(void);
void init_heart_rate(void);
bool get_hr_spo2(uint32_t* p_hr, uint32_t* p_spo2);
void get_stress_level(char *p_stress);
///////////////////////////////////////// | End of Header File | ///////////////////////////////////////////

#endif /* MAX30102_H_ */