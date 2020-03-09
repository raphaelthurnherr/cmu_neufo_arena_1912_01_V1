/**
 * \file mcp4728.h
 * \brief MCP4728 4 channels Digital analog converter with eeprom memory driver
 *  I2C default address: 0x60 (7 bit)
 * \author Raphael Thurnherr
 * \version 0.1
 * \date 05.09.2019
 *
 * Library to setup and drive the DAC converter MCP4728
 * 
 */

#ifndef MCP4728_H
#define MCP4728_H

/**
 * \struct mcp4725 [mcp4725.h] Configuration structure definition
 */

typedef struct mcp4728{
    char deviceName[25];                        // Device Name of IC
    unsigned char deviceAddress;                // Bus device address
    unsigned int vref_mv;                       // VoltageReference value in mV (2.7..5.5)
} device_mcp4728;

/**
 * \brief MCP4728 driver initialization
 * \param pointer on the configuration structure
 * \return code error
 */
extern int mcp4728_init(device_mcp4728 *mcp4728config);        // MCP4728 driver initialization

/**
 * \brief MCP4728 set output voltage in mV
 * \param pointer on the configuration structure
 * \return code error
 */
extern int mcp4728_setDACOutput_mV(device_mcp4728 *mcp4728config,  unsigned char channel, int value_mv);

/**
 * \brief MCP4728 set output voltage by 12 bit value
 * \param pointer on the configuration structure
 * \return code error
 */
extern int mcp4728_setDAC_12bitValue(device_mcp4728 *mcp4728config,  unsigned char channel, int value);

extern char actuator_setAnalogOutput(device_mcp4728 *mcp4728config, unsigned char channel, int value);

#endif /* MCP4728_H */