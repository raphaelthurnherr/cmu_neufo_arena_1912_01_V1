#include <Arduino.h>
#include <Wire.h>
#include "cmu_ws_1912_01_V1_board.h"


board_1912_01_V01::board_1912_01_V01(void){
    Wire.begin();                       // Initiate the Wire library for I2C

    dev_pca9629.deviceAddress = 0x23;
    dev_pca9629.pulsesWidth_ms = 1;
}

void board_1912_01_V01::begin(void){
    pca9629_init(&dev_pca9629);
}

void  board_1912_01_V01::stepperRotation(char motor, char speed, unsigned int steps){
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
    default: selectedMotor = dev_pca9629; break;
}

    actuator_setStepperSpeed(&selectedMotor, speed);
    actuator_setStepperStepAction(&selectedMotor, direction, steps);
}


int  board_1912_01_V01::getStepperState(unsigned char motorNumber){
    device_pca9629 selectedMotor;

    switch (motorNumber){
        case 0: selectedMotor = dev_pca9629; break;
        default: selectedMotor = dev_pca9629; break;
    }

    int state = (actuator_getStepperState(&selectedMotor) & 0x80);
    return state;
}