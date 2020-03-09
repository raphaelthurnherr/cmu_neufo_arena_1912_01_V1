/**
 * \file CONFIG_1921_01_V1.h
 * \brief CONFIG FILE FOR DEVICE ADDRESS DEFINITION 
 * \author Raphael Thurnherr
 * \version 0.1
 * \date 07.01.2020
 */

#ifndef CONFIG_1921_01_V1_H
#define CONFIG_1921_01_V1_H
#endif

#ifndef _1921_01_V1_TYPE_NOTDEFAULT
    #define IC2_TCA9544A_ADR    0x70
    #define IC3_ADS1115_ADR     0x48

    #define IC5_MCP4728_ADR     0x60
    #define IC5_MCP4728_VREF    3300      // External Vref (VDD)

    #define IC7_MCP23017_ADR    0x20
    #define IC7_GPIO_DIRECTION  0xFF0F    // 0=output, 1=input
    #define IC7_GPIO_PULLUP     0x0000    // 0=disable, 1=enable
#else
    #ifdef _1921_01_V1_TYPE_A0
        #define IC2_TCA9544A_ADR    0x70
        
        #define IC3_ADS1115_ADR     0x48
        #define IC5_MCP4728_ADR     0x60
        #define IC5_MCP4728_VREF    3300      // External Vref (VDD)

        #define IC7_MCP23017_ADR    0x20   
        #define IC7_GPIO_DIRECTION  0xff0f    // 0=output, 1=input
        #define IC7_GPIO_PULLUP     0x0000    // 0=disable, 1=enable
    #endif
#endif

