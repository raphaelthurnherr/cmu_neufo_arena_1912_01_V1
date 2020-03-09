#ifndef cmu_ws_1921_01_V1_board_h
#define cmu_ws_1921_01_V1_board_h

#include <Arduino.h>

// Low level device hardware library
#include "device_drivers/src/tca9544a.h"
#include "device_drivers/src/ads111x.h" 
#include "device_drivers/src/mcp230xx.h"
#include "device_drivers/src/mcp4728.h"

class board_1921_01_V01{
    public:
        board_1921_01_V01(void);

        void begin(void);
        void selectI2Cchannel(unsigned char channel);
        void disableI2Cchannels(void);

        int getVoltage(unsigned char channel);
        char getDigitalInput(unsigned char channel);
        char setDigitalOutput(unsigned char channel, unsigned char value);
        int setAnalogOutput(unsigned char channel, int value);

    private:
        device_ads111x dev_ads111x;
        device_mcp230xx dev_mcp23017;
        device_tca9544a dev_tca9544xa;
        device_mcp4728 dev_mcp4728;

};
    
#endif