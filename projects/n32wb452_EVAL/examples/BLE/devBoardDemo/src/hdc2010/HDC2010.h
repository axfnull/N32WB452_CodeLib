/*
        HDC2010.h
        Created By: Brandon Fisher, August 1st 2017

        This code is release AS-IS into the public domain, no guarantee or warranty is given.

        Description: This header file accompanies HDC2010.cpp, and declares all methods, fields,
        and constants used in the source code.
*/

#ifndef __HDC2010_H__
#define __HDC2010_H__

#include "n32wb452.h"

//  Constants for setting measurement resolution
#define FOURTEEN_BIT 0
#define ELEVEN_BIT   1
#define NINE_BIT     2

//  Constants for setting sensor mode
#define TEMP_AND_HUMID  0
#define TEMP_ONLY       1
#define HUMID_ONLY      2
#define ACTIVE_LOW      0
#define ACTIVE_HIGH     1
#define LEVEL_MODE      0
#define COMPARATOR_MODE 1

//  Constants for setting sample rate
#define MANUAL       0
#define TWO_MINS     1
#define ONE_MINS     2
#define TEN_SECONDS  3
#define FIVE_SECONDS 4
#define ONE_HZ       5
#define TWO_HZ       6
#define FIVE_HZ      7

// Define Register Map
#define TEMP_LOW           0x00
#define TEMP_HIGH          0x01
#define HUMID_LOW          0x02
#define HUMID_HIGH         0x03
#define INTERRUPT_DRDY     0x04
#define TEMP_MAX           0x05
#define HUMID_MAX          0x06
#define INTERRUPT_CONFIG   0x07
#define TEMP_OFFSET_ADJUST 0x08
#define HUM_OFFSET_ADJUST  0x09
#define TEMP_THR_L         0x0A
#define TEMP_THR_H         0x0B
#define HUMID_THR_L        0x0C
#define HUMID_THR_H        0x0D
#define CONFIG             0x0E
#define MEASUREMENT_CONFIG 0x0F
#define MID_L              0xFC
#define MID_H              0xFD
#define DEVICE_ID_L        0xFE
#define DEVICE_ID_H        0xFF

// Define Register config

//#define INTERRUPT_DRDY     0x04
//bit[7]:DataReady bit status       
#define  DRDY_STATUS_NOT_READAY      0x00
#define  DRDY_STATUS_READAY          0x80              
//bit[6]:Temperature threshold HIGH Interrupt status
#define  HH_STATUS_NO_INT            0x00
#define  HH_STATUS_INT               0x40            
//bit[5]:Temperature threshold LOW Interrupt status
#define  HL_STATUS_NO_INT            0x00
#define  HL_STATUS_INT               0x20            
//bit[4]:Humidity threshold HIGH Interrupt status
#define  TH_STATUS_NO_INT            0x00
#define  TH_STATUS_INT               0x10            
//bit[3]:Humidity threshold LOW Interrupt status
#define  TL_STATUS_NO_INT            0x00
#define  TL_STATUS_INT               0x08

//#define INTERRUPT_CONFIG   0x07
//bit[7]:DataReady Interrupt mask
#define  DRDY_MASK_ENABLE            0x80
#define  DRDY_MASK_DISABLE           0x00
//bit[6]:Temperature threshold HIGH Interrupt mask
#define  TH_MASK_ENABLE              0x00       
#define  TH_MASK_DISABLE             0x40
//bit[5]:Temperature threshold LOW Interrupt mask
#define  TL_MASK_ENABLE              0x00
#define  TL_MASK_DISABLE             0x20
//bit[4]:Humidity threshold HIGH Interrupt mask
#define  HH_MASK_ENABLE              0x00
#define  HH_MASK_DISABLE             0x10
//bit[3]:Humidity threshold HIGH Interrupt mask
#define  HL_MASK_ENABLE              0x00
#define  HL_MASK_DISABLE             0x08

//#define CONFIG             0x0E
//bit[7]:
#define SOFT_RES_NORMAL             0x00
#define SOFT_RES_SOFT               0x80
//bit[6:4]:Output Data Rate       
#define ODR_NO                      0x00         //000
#define ODR_1_120HZ                 0x10         //001
#define ODR_1_60HZ                  0x20         //010 
#define ODR_0_1HZ                   0x30         //011
#define ODR_0_2HZ                   0x40         //100
#define ODR_1HZ                     0x50         //101
#define ODR_2HZ                     0x60         //110
#define ODR_5HZ                     0x70         //111
//bit[3]:
#define HEAT_EN_ON                  0x08
#define HEAT_EN_OFF                 0x00
//bit[2]:DRDY/INT_EN pin configuration
#define DRDY_OR_INT_EN_ENABLE       0x04
#define DRDY_OR_INT_EN_HIGH_Z       0x00
//bit[1]:Interrupt polarity       
#define INT_POL_LOW                 0x00
#define INT_POL_HIGH                0x02
//bit[0]:Interrupt mode
#define INT_MODE_LEVEL              0x00
#define INT_MODE_COMPARAM           0x01

//#define MEASUREMENT_CONFIG 0x0F
//bit[7:6]:Temperature resolution
#define  TEM_RES_14BIT              0x00           //00
#define  TEM_RES_11BIT              0x40           //01
#define  TEM_RES_9BIT               0x80           //10
//bit[5:4]:Humidity resolution
#define  HUM_RES_14BIT              0x00           //00
#define  HUM_RES_11BIT              0x10           //01
#define  HUM_RES_9BIT               0x20           //10
//bit[3]:res          
//bit[2:1]:Measurement configuration
#define  MEAS_CONF_TEM_HUM          0x00           //00
#define  MEAS_CONF_TEM_ONLY         0x02           //01
#define  MEAS_CONF_HUM_ONLY         0x04           //10
//bit[0]:Measurement trigger
#define  MEAS_TRIG_START            0x01
#define  MEAS_TRIG_NO_ACTION        0x00


float readTemp(void);                  // Returns the temperature in degrees C
float readHumidity(void);              // Returns the relative humidity
void enableHeater(void);               // Enables the heating element
void disableHeater(void);              // Disables the heating element
void setLowTemp(float temp);           // Sets low threshold temperature (in c)
void setHighTemp(float temp);          // Sets high threshold temperature (in c)
void setHighHumidity(float humid);     // Sets high Humiditiy threshold
void setLowHumidity(float humid);      // Sets low Humidity threshold
float readLowHumidityThreshold(void);  // Returns contents of low humidity threshold register
float readHighHumidityThreshold(void); // Returns contents of high humidity threshold register
float readLowTempThreshold(void);      // Returns contents of low temperature threshold register (in C)
float readHighTempThreshold(void);     // Returns contents of high temperature threshold register (in C)
void triggerMeasurement(void);         // Triggers a manual temperature/humidity reading
void reset(void);                      // Triggers a software reset
void enableInterrupt(void);            // Enables the interrupt/DRDY pin
void disableInterrupt(void);           // Disables the interrupt/DRDY pin (High Z)
uint8_t readInterruptStatus(void);     // Reads the status of the interrupt register
void clearMaxTemp(void);               // Clears the Maximum temperature register
void clearMaxHumidity(void);           // Clears the Maximum humidity register
float readMaxTemp(void);               // Reads the maximum temperature register
float readMaxHumidity(void);           // Reads the maximum humidity register
void enableThresholdInterrupt(void);   // Enables high and low temperature/humidity interrupts
void disableThresholdInterrupt(void);  // Disables high and low temperature/humidity interrupts
void enableDRDYInterrupt(void);        // Enables data ready interrupt
void disableDRDYInterrupt(void);       // Disables data ready interrupt

/* Sets Temperature & Humidity Resolution, 3 options
0 - 14 bit
1 - 11 bit
2 - 9 bit
default - 14 bit                            */
void setTempRes(int resolution);
void setHumidRes(int resolution);

/* Sets measurement mode, 3 options
0 - Temperature and Humidity
1 - Temperature only
2 - Humidity only
default - Temperature & Humidity            */
void setMeasurementMode(int mode);

/* Sets reading rate, 8 options
0 - Manual
1 - reading every 2 minutes
2 - reading every minute
3 - reading every ten seconds
4 - reading every 5 seconds
5 - reading every second
6 - reading at 2Hz
7 - reading at 5Hz
default - Manual        */
void setRate(int rate);

/* Sets Interrupt polarity, 2 options
0 - Active Low
1 - Active High
default - Active Low            */
void setInterruptPolarity(int polarity);

/* Sets Interrupt mode, 2 options
0 - Level sensitive
1 - Comparator mode
default - Level sensitive   */
void setInterruptMode(int polarity);

void openReg(uint8_t reg);                // Points to a given register
uint8_t readReg(uint8_t reg);             // Reads a given register, returns 1 byte
void writeReg(uint8_t reg, uint8_t data); // Writes a byte of data to one register

bool checkHDC2010Chip(uint16_t *dev_id);
void readHDC2010_16bitData(uint8_t addr, uint16_t *data);
void hdc2010_init_reg(void);

#endif
