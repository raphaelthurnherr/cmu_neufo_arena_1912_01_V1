#include <Arduino.h>
#include <Wire.h>
#include "CONFIG_1921_01_V1.h"
#include "cmu_ws_1921_01_V1_board.h"

board_1921_01_V01::board_1921_01_V01(void){
    Wire.begin();                       // Initiate the Wire library for I2C

    dev_tca9544xa.deviceAddress = IC2_TCA9544A_ADR;
    
    dev_mcp23017.deviceAddress = IC7_MCP23017_ADR;
    //dev_mcp23017.invertedInput = IC7_GPIO_INVERTED;                 // 0=disable, 1=enable
    dev_mcp23017.gpioDirection = IC7_GPIO_DIRECTION;                // 0=output, 1=input
    dev_mcp23017.pullupEnable  = IC7_GPIO_PULLUP;                   // 0=disable, 1=enable 

    dev_ads111x.deviceAddress = IC3_ADS1115_ADR;
    
    dev_mcp4728.deviceAddress = IC5_MCP4728_ADR;
    dev_mcp4728.vref_mv = IC5_MCP4728_VREF;
}

void board_1921_01_V01::begin(void){
    
    tca9544a_init(&dev_tca9544xa);
    tca9544a_selectChannel(&dev_tca9544xa, 0);

    // Setting up the mcp23017 GPIO extender
    mcp23017_init(&dev_mcp23017);            
    Serial.println("CONFIG GPIO ");

    ads111x_init(&dev_ads111x);   
    mcp4728_init(&dev_mcp4728);
}

void board_1921_01_V01::selectI2Cchannel(unsigned char channel){
    tca9544a_selectChannel(&dev_tca9544xa, channel);
}

void board_1921_01_V01::disableI2Cchannels(void){
    tca9544a_disableChannels(&dev_tca9544xa);
}

int  board_1921_01_V01::getVoltage(unsigned char channel){
    int voltage_mV;
    voltage_mV = actuator_getVoltage(&dev_ads111x, channel);

    return voltage_mV;
}

char  board_1921_01_V01::getDigitalInput(unsigned char channel){
    char state;
    state = actuator_getDigitalInput(&dev_mcp23017, channel);

    return state;
}

char  board_1921_01_V01::setDigitalOutput(unsigned char channel, unsigned char value){
    char result = -1;
    actuator_setDigitalOutput(&dev_mcp23017, channel, value);

    return result;
}

int  board_1921_01_V01::setAnalogOutput(unsigned char channel, int value){
    int result = -1;
    actuator_setAnalogOutput(&dev_mcp4728, channel, value);

    return result;
}




/*
void Kehops::setPWM(unsigned char pwmNb, unsigned char value){
    pca9685_setPWMdutyCycle(&dev_pca9685, pwmNb, value);
}

void  Kehops::stepperRotation(char motor, char speed, unsigned int steps){
    int direction;
    device_pca9629 selectedMotor;

    if(speed > 0)
        direction = 1;
    else 
        if(speed < 0){
            direction = -1;
            speed *= -1;
        }

switch (motor){
    case 0: selectedMotor = dev_pca9629; break;
    case 1: selectedMotor = dev_motor2; break;
    default: selectedMotor = dev_pca9629; break;
}

    actuator_setStepperSpeed(&selectedMotor, speed);
    actuator_setStepperStepAction(&selectedMotor, direction, steps);
}


int  Kehops::getStepperState(unsigned char motorNumber){
    device_pca9629 selectedMotor;

    switch (motorNumber){
        case 0: selectedMotor = dev_pca9629; break;
        case 1: selectedMotor = dev_motor2; break;
        default: selectedMotor = dev_pca9629; break;
    }

    int state = (actuator_getStepperState(&selectedMotor) & 0x80);
    return state;
}

*/