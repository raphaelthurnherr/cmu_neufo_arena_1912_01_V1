/**
 * \file tca9544a.cpp
 * \brief tca9544a I2C Switch driver
 *  I2C default address: 0x70 (7 bit)
 * \author Raphael Thurnherr
 * \version 0.1
 * \date 11.12.2019
 *
 * Library to setup and drive the 8 channel I2C switch
 *
 */

#ifndef I2CSIMU

#include "tca9544a.h"
#include "arduino-i2c.h"

/**
 * \brief tca9544a driver initialization
 * \param pointer on the configuration structure
 * \return code error
 */
int tca9544a_init(device_tca9544a *tca9544aconfig){
    int err =0;    
    unsigned char regData[32];
    unsigned char data[32];
    unsigned char deviceAddress = tca9544aconfig->deviceAddress;
 
    // Prepare register for  all channel default OFF
    data[0] = 0x05;
    
    // Test read
    err += i2c_readRaw(0, deviceAddress, regData, 1);   
    
    // Set all channels ON/OFF
    err += i2c_writeRaw(0, deviceAddress, data, 1);

    if(err){
       // printf("Kehops I2C tca9544a device initialization with %d error\n", err);
    }
    return err;
}


/**
 * \brief tca9544a set channel state
 * \param pointer on the configuration structure
 * \param channel selection
 * \param channel state (ON/OFF)
 * \return code error
 */

int tca9544a_selectChannel(device_tca9544a *tca9544aconfig, unsigned char channel){
    char err =0;
    unsigned char deviceAddress = tca9544aconfig->deviceAddress;
    
    unsigned char controlRegisterValue[32];
    
        // Load the register value for channel selection
        switch(channel){
            case 0 : controlRegisterValue[0] = 0x04; break;     
            case 1 :  controlRegisterValue[0] = 0x05;; break;
            case 2 :  controlRegisterValue[0] = 0x06; break;
            case 3 :  controlRegisterValue[0] = 0x07; break;

            default: break;
        }

    err += i2c_writeRaw(0, deviceAddress, controlRegisterValue, 1);
        
    return err;
}

int tca9544a_disableChannels(device_tca9544a *tca9544aconfig){
    char err =0;
    unsigned char deviceAddress = tca9544aconfig->deviceAddress;

     // Load the register command value for disable
    err += i2c_writeRaw(0, deviceAddress, 0x00, 1);

    return err;
}

#endif