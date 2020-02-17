#include <Arduino.h>
#include <Wire.h>
#include "cmu_ws_1912_01_V1_board.h"


board_1912_01_V01::board_1912_01_V01(void){
    Wire.begin();                       // Initiate the Wire library for I2C

    CHANNEL_A_MOTOR.deviceAddress = 0x21;
    CHANNEL_A_MOTOR.pulsesWidth_ms = 1;

    CHANNEL_B_MOTOR.deviceAddress = 0x22;
    CHANNEL_B_MOTOR.pulsesWidth_ms = 1;

    CHANNEL_C_MOTOR.deviceAddress = 0x23;
    CHANNEL_C_MOTOR.pulsesWidth_ms = 1;
}

void board_1912_01_V01::begin(void){
    pca9629_init(&CHANNEL_A_MOTOR);
    pca9629_init(&CHANNEL_B_MOTOR);
    pca9629_init(&CHANNEL_C_MOTOR);
}

void  board_1912_01_V01::stepperRotation(char motor, char speed, int steps){
    int direction;
    device_pca9629 *selectedMotor;

    if(steps > 0)
        direction = 1;
    else 
        if(steps < 0){
            direction = -1;
            steps *= -1;
        }

switch (motor){
    case MOTOR_A: selectedMotor = &CHANNEL_A_MOTOR; break;
    case MOTOR_B: selectedMotor = &CHANNEL_B_MOTOR; break;
    case MOTOR_C: selectedMotor = &CHANNEL_C_MOTOR; break;
    default: selectedMotor = &CHANNEL_A_MOTOR; break;
}

    actuator_setStepperSpeed(selectedMotor, speed);
    actuator_setStepperStepAction(selectedMotor, direction, steps);
}


int  board_1912_01_V01::getStepperState(unsigned char motorNumber){
    device_pca9629 *selectedMotor;

    switch (motorNumber){
    case 0: selectedMotor = &CHANNEL_A_MOTOR; break;
    case 1: selectedMotor = &CHANNEL_B_MOTOR; break;
    case 2: selectedMotor = &CHANNEL_C_MOTOR; break;
    default: selectedMotor = &CHANNEL_A_MOTOR; break;
    }

    int state = (actuator_getStepperState(selectedMotor) & 0x80);
    return state;
}