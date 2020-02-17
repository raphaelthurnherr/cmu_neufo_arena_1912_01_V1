/**
 * \file tca9544a.h
 * \brief tca9544a I2C Switch driver
 *  I2C default address: 0x70 (7 bit)
 * \author Raphael Thurnherr
 * \version 0.1
 * \date 11.12.2019
 *
 * Library to setup and drive the 4 channel I2C switch
 * 
 */


#ifndef tca9544a_H
#define tca9544a_H

#define STATE_OFF  0            // OFF state
#define STATE_ON  !STATE_OFF    // ON state

/**
 * \struct tca9544a [tca9544a.h] Configuration structure definition
 */

typedef struct tca9544a{
    char deviceName[25];                        // Device Name of IC
    unsigned char deviceAddress;                // Bus device address
} device_tca9544a;

/**
 * \brief tca9544a driver initialization
 * \param pointer on the configuration structure
 * \return code error
 */
extern int tca9544a_init(device_tca9544a *tca9544aconfig);        // tca9544a driver initialization

/**
 * \brief tca9544a set channel state
 * \param pointer on the configuration structure
 * \param channel selection
 * \param channel state (ON/OFF/XON)
 * \return code error
 */
extern int tca9544a_selectChannel(device_tca9544a *tca9544aconfig, unsigned char channel);
extern int tca9544a_disableChannels(device_tca9544a *tca9544aconfig);

#endif /* tca9544a_H */