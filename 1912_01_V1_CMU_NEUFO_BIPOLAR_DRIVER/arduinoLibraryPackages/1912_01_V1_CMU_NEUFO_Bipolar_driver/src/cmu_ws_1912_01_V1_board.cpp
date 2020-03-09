#include <Arduino.h>
#include <Wire.h>
#include "CONFIG_1912_01_V1.h"
#include "cmu_ws_1912_01_V1_board.h"


board_1912_01_V01::board_1912_01_V01(void){
    Wire.begin();                       // Initiate the Wire library for I2C

    CHANNEL_A_MOTOR.deviceAddress = IC1_PCA9629A_ADR;
    CHANNEL_A_MOTOR.pulsesWidth_ms = IC1_PULSE_WIDTH_MS;
    CHANNEL_A_MOTOR.bipolar_mode = IC1_MOTOR_MODE_BIPOLAR;

    CHANNEL_B_MOTOR.deviceAddress = IC2_PCA9629A_ADR;
    CHANNEL_B_MOTOR.pulsesWidth_ms = IC2_PULSE_WIDTH_MS;
    CHANNEL_B_MOTOR.bipolar_mode = IC2_MOTOR_MODE_BIPOLAR;

    CHANNEL_C_MOTOR.deviceAddress = IC3_PCA9629A_ADR;
    CHANNEL_C_MOTOR.pulsesWidth_ms = IC3_PULSE_WIDTH_MS;
    CHANNEL_C_MOTOR.bipolar_mode = IC3_MOTOR_MODE_BIPOLAR;
}

void board_1912_01_V01::begin(void){
    pca9629_init(&CHANNEL_A_MOTOR);
    pca9629_init(&CHANNEL_B_MOTOR);
    pca9629_init(&CHANNEL_C_MOTOR);
}

void  board_1912_01_V01::stepperRotation(char motor, char speed, int steps){
    int direction;
    device_pca9629 *selectedMotor;

    if(speed > 0)
        direction = 1;
    else 
        if(speed < 0){
            direction = -1;
            speed*=-1;
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

int  board_1912_01_V01::setStepperDriveMode(char motorNumber, unsigned char driveMode){

    device_pca9629 *selectedMotor;

    if(driveMode == 0)
        driveMode = 1;      // Two phase drive (full step)
    else driveMode = 2;     // Half step drive

    switch (motorNumber){
    case 0: selectedMotor = &CHANNEL_A_MOTOR; break;
    case 1: selectedMotor = &CHANNEL_B_MOTOR; break;
    case 2: selectedMotor = &CHANNEL_C_MOTOR; break;
    default: selectedMotor = &CHANNEL_A_MOTOR; break;
    }

    actuator_setStepperDriveMode(selectedMotor, driveMode);
    return (0);
}