#ifndef cmu_ws_1912_01_V1_board_h
#define cmu_ws_1912_01_V1_board_h

#include <Arduino.h>

//Low level device hardware library
#include "pca9629.h"


class board_1912_01_V01{
    public:
        board_1912_01_V01(void);
        void begin(void);
        int getStepperState(unsigned char motorNumber);
        void stepperRotation(char motor, char speed, unsigned int steps);

    protected:
    device_pca9629 dev_pca9629;
};

#endif