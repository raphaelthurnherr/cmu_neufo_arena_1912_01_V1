/**
 * \file mcp4728.h
 * \brief MCP4728 4 channels Digital analog converter with eeprom memory driver
 *  I2C default address: 0x60 when ADR connected to GND
 *  I2C default address: 0x61 when ADR connected to VCC
 * \author Raphael Thurnherr
 * \version 0.1
 * \date 05.09.2019
 *
 * Library to setup and drive the DAC converter MCP4728
 * 
 */

// user Definitions
#define RESOLUTION  4095

#include <Arduino.h>
#include "mcp4728.h"
#include "arduino-i2c.h"

#include <stdio.h>
#include <math.h>

#ifndef I2CSIMU


/**
 * \brief MCP4728 driver initialization
 * \param pointer on the configuration structure
 * \return code error
 */

int mcp4728_init(device_mcp4728 *mcp4728config){
    int err =0;    
    unsigned char mcp4728_regData[32];
    unsigned char deviceAddress = mcp4728config->deviceAddress;

    // Get the DAC register setting (8 bit), DAC Register data (16 bit), EEPROM DATA(16 bit)   
    err += i2c_readRaw(0, deviceAddress, mcp4728_regData , 5);

    if(err){
//        printf("Kehops I2C MCP4728 device initialization with %d error\n", err);
    }
    return err;
}


/**
 * \brief MCP4728 set DAC output voltage in mV
 * \param pointer on the configuration structure
 * \param channel, specify the channel to set state
 * \param value_mv, mV value for output
 * \return code error
 */

int mcp4728_setDACOutput_mV(device_mcp4728 *mcp4728config, unsigned char channel, int value_mv){
    unsigned char err =0;
    unsigned char deviceAddress = mcp4728config->deviceAddress;
    unsigned int vref = mcp4728config->vref_mv;
    unsigned int regData =0;
    float resolution_mv;
    
    resolution_mv = (float)vref / RESOLUTION;

    regData = round((float)value_mv / resolution_mv);
    //regData = (float)value_mv / resolution_mv;

    unsigned char data[32];
    
    // Write only DAC Register: (C2, C1, C0) (W1, W0)= (0,1,0) (1,1) and  POWERDOWN =0;
    data[0] = 0x58;

    // Select DAC Register
    data[0] |= (channel<<1);
    
    // Force UDAC to 0
    data[0] &= 0xFE;
            
    
    // Loading buffer with 12bit register value
    data[1] = ((regData & 0x0f00) >> 8);             // MSB
    data[2] = (regData & 0x00ff);                    // LSB
    
    //data[1] |= 0x80;                        // VREF=1 (internal 2.048V), NO PULL DOWN, GAIN = 1        

    err += i2c_writeRaw(0, deviceAddress, data, 3);
    
    //printf("Write I2C mcp4728_setDACOutput_mV:     valueMV: %d     Address:   0x%2x", value_mv,  deviceAddress );
    err++;
    return err;
}

/**
 * \brief MCP4728 set DAC output 12bit value
 * \param pointer on the configuration structure
 * \param channel, specify the channel to set state
 * \param value (12bit), mV value for output
 * \return code error
 */

int mcp4728_setDAC_12bitValue(device_mcp4728 *mcp4728config,  unsigned char channel, int value){
    unsigned char err =0;
    unsigned char deviceAddress = mcp4728config->deviceAddress;
    unsigned char data[32];
    
        // Write only DAC Register: (C2, C1, C0) (W1, W0)= (0,1,0) (1,1) and  POWERDOWN =0;
    data[0] = 0x58;
    
    // Select DAC Register
    data[0] |= (channel<<1);
    
    // Force UDAC to 0 (VOUT update)
    data[0] &= 0xFE;

    // Loading buffer with 12bit register value
    data[1] = ((value & 0x0f00) >> 8);        // MSB
    data[2] = value & 0x00ff;                 // LSB

    data[1] |= 0x80;                        // VREF=1 (internal 2.048V), NO PULL DOWN, GAIN = 1

    err += i2c_writeRaw(0, deviceAddress, data, 3);
    
    err++;
    return err;
}



char actuator_setAnalogOutput(device_mcp4728 *mcp4728config, unsigned char channel, int value){

    // USE DRIVER FOR MCP4725
    /*
    if(!strcmp(kehopsActuators.aout[aoutID].hw_driver.type, DRIVER_MCP4725)){
        ptrDev = getDriverConfig_ptr(DRIVER_MCP4725, kehopsActuators.aout[aoutID].hw_driver.name);
        if(ptrDev>=0){
    #ifdef INFO_DEBUG
            printf("SET ANALOG VALUE FROM <%s> DRIVERS:  NAME:%s TYPE:%s I2C add: 0x%2x    aout_id: %d     channel: %d     value: %d\n",DRIVER_MCP4725, kehopsActuators.aout[aoutID].hw_driver.name, kehopsActuators.aout[aoutID].hw_driver.type, dev_mcp4725[ptrDev].deviceAddress, aoutID, channel, value);        
    #endif
            if(dev_mcp4725[ptrDev].deviceAddress > 0)
                mcp4725_setDACOutput_mV(&dev_mcp4725[ptrDev], value);
            else
                {
                #ifdef INFO_BUS_DEBUG                
                printf("#! Function [actuator_setAnalogValue] -> I2C Error: Bad address or device not connected\n");
                #endif
            }
            
        }else 
            printf ("#! Function [actuator_setAnalogValue] -> Unknown driver name: %s\n", kehopsActuators.aout[aoutID].hw_driver.name);
    }
    
    */
    mcp4728_setDACOutput_mV(mcp4728config, channel, value);
    //mcp4728_setDAC_12bitValue(mcp4728config, channel, value);
    /*
    // USE DRIVER FOR MCP4728
    if(!strcmp(kehopsActuators.aout[aoutID].hw_driver.type, DRIVER_MCP4728)){
        ptrDev = getDriverConfig_ptr(DRIVER_MCP4728, kehopsActuators.aout[aoutID].hw_driver.name);
        if(ptrDev>=0){
    #ifdef INFO_DEBUG
            printf("SET ANALOG VALUE FROM <%s> DRIVERS:  NAME:%s TYPE:%s I2C add: 0x%2x    aout_id: %d     channel: %d     value: %d\n",DRIVER_MCP4725, kehopsActuators.aout[aoutID].hw_driver.name, kehopsActuators.aout[aoutID].hw_driver.type, dev_mcp4725[ptrDev].deviceAddress, aoutID, channel, value);        
    #endif
            int channel = kehopsActuators.ain[aoutID].hw_driver.attributes.device_channel;
            if(dev_mcp4728[ptrDev].deviceAddress > 0)
                mcp4728_setDACOutput_mV(&dev_mcp4728[ptrDev], channel, value);
            else
                {
                #ifdef INFO_BUS_DEBUG                
                printf("#! Function [actuator_setAnalogValue] -> I2C Error: Bad address or device not connected\n");
                #endif
            }
            
        }else 
            printf ("#! Function [actuator_setAnalogValue] -> Unknown driver name: %s\n", kehopsActuators.aout[aoutID].hw_driver.name);
    }    
    
    // SUBDRIVER ACTION Check if driver IC need a subdriver and make a post-action if required
    subDriver_onDeactivate(kehopsActuators.aout[aoutID].hw_driver.name);    
    */

   return (0);
}

#endif