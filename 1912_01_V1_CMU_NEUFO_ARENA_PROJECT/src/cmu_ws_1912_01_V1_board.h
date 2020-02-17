#ifndef cmu_ws_1912_01_V1_board_h
#define cmu_ws_1912_01_V1_board_h

#include <Arduino.h>
//Low level device hardware library
#include "device_drivers/src/pca9629.h"

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2

class board_1912_01_V01{
    public:
        board_1912_01_V01(void);
        void begin(void);
        int getStepperState(unsigned char motorNumber);
        void stepperRotation(char motor, char speed, int steps);

    protected:
    device_pca9629 CHANNEL_A_MOTOR;
    device_pca9629 CHANNEL_B_MOTOR;
    device_pca9629 CHANNEL_C_MOTOR;
};

#endif