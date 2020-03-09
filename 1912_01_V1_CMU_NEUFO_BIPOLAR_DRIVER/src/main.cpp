#include <Arduino.h>

#include "cmu_ws_1912_01_V1_board.h"
#include "cmu_ws_1921_01_V1_board.h"


// Define the default motor speed and steps for run from BNC trigger
#define DEFAULT_MOTOR_SPEED 25
#define DEFAULT_MOTOR_STEPS 200


// Main board 1921 - IC7 digital output bit definition for driver enable
#define GATE_A_ENABLE_BIT 6
#define GATE_B_ENABLE_BIT 5
#define GATE_C_ENABLE_BIT 4

//  Frame slots for commands
#define SYNC_SOF  0
#define MCMD  1
#define SPEED 2
#define STEPSCOUNT 3
#define SYNC_EOF  5

// Main board 1921 - IC7 digital input bit definition on BNC INPUTS 
#define MOTOR_B_CMD_TRIGGER 8
#define MOTOR_C_CMD_TRIGGER 13

// Boards declaration
board_1921_01_V01 arduino_1921_board;
board_1912_01_V01 motor_1912_board;

// Serial command read buffer
unsigned char myMotorCommand[8];
char motor_B_last_cmd_input_state;
char motor_C_last_cmd_input_state;

// Data commands
char motorSelected=0;
char motorSens=0;
char motorSpeed=0;
int motorSteps=0;
unsigned char motorStepModeHalf=0;
unsigned char motorRunCommand=0;

//generic variable
int i;

// Functions déclaration
char get_serial_input_frame(unsigned char * myMotorCommand);    // Get the serial data, wait for 6bytes (frame lenght)

// Arduino setup
void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:

  // Main board 1921 initialization
  arduino_1921_board.begin();
  // Motor board 1912 initialization
  motor_1912_board.begin();

  // Enable the motors power H-BRIDGES
  arduino_1921_board.setDigitalOutput(GATE_C_ENABLE_BIT,1);     // Gate C enable
  arduino_1921_board.setDigitalOutput(GATE_B_ENABLE_BIT,1);     // Gate B enable
  arduino_1921_board.setDigitalOutput(GATE_A_ENABLE_BIT,1);     // Gate A enable

  // Initial message
  Serial.write("HELLO ! I WAITING FOR SERIAL COMMANDS, PLEASE SEE DOCUMENTATION...");
  delay(10);

}


// Main ARDUINO LOOP

void loop() {


  // Get input the BNC input state and make action if triggered, 
  // reset trigget when BNC input go to lower state
  
  //  USE BNC INPUT TRIGGER 0 for motor B(=motor 1)
  if(arduino_1921_board.getDigitalInput(MOTOR_B_CMD_TRIGGER)){
    // Check if trigger was cleared
    if(!motor_B_last_cmd_input_state){
      motor_1912_board.stepperRotation(0, DEFAULT_MOTOR_SPEED, DEFAULT_MOTOR_STEPS);  
      motor_B_last_cmd_input_state = 1;
    }
  }else motor_B_last_cmd_input_state = 0;

  //  USE BNC INPUT TRIGGER 1 for motor C(=motor 2)
  if(arduino_1921_board.getDigitalInput(MOTOR_C_CMD_TRIGGER)){
    // Check if trigger was cleared
    if(!motor_C_last_cmd_input_state){
      motor_1912_board.stepperRotation(2, DEFAULT_MOTOR_SPEED, DEFAULT_MOTOR_STEPS);
      motor_C_last_cmd_input_state = 1;
    }
  }else motor_C_last_cmd_input_state = 0;

  // Check the SOF and EOF before execute commands
  if(get_serial_input_frame(myMotorCommand)){

    // Check SOF and EOF bytes
    if((myMotorCommand[SYNC_SOF] == (unsigned char) 0xAA) && (myMotorCommand[SYNC_EOF] == (unsigned char) 0xBB)){

        // Get the command bit
        motorSteps = 0;
        motorSelected = myMotorCommand[MCMD] & 0x03;
        motorSens = myMotorCommand[MCMD] & 0x20;
        motorStepModeHalf = myMotorCommand[MCMD] & 0x10;
        motorRunCommand = (myMotorCommand[MCMD] & 0x0C) >> 2;
        
        // Get the speed and step data
        motorSpeed = myMotorCommand[SPEED];
        motorSteps = myMotorCommand[STEPSCOUNT];
        motorSteps = (motorSteps << 8) |  myMotorCommand[STEPSCOUNT+1];

        // Format the speed for function (speed < 0: CCW, speed > 0 :CW)
        if(motorSens)
          motorSpeed *= -1;

        // Prepare the run command for function
        switch(motorRunCommand){
          case 0x00 : motorSteps = 0; break;                    // Motor Stop
          case 0x01 : ; break;                                  // Motor run step count
          case 0x02 : motorSteps = -1; break;                   // Motor run continuous
          default: ; break;
        }
/*
        Serial.write("MOTOR: ");
        Serial.print(motorSelected, DEC);
        Serial.write("  PAS: ");
        Serial.print(motorSteps, DEC);
        Serial.write("  SPEED: ");
        Serial.print(motorSpeed, DEC);
        Serial.write("  SENS: ");
        Serial.print(motorSens, DEC);
*/

        // Set the driver motor mode (two-phase of half-step)
        motor_1912_board.setStepperDriveMode(motorSelected, motorStepModeHalf);
        motor_1912_board.stepperRotation(motorSelected, motorSpeed, motorSteps);

        // Clear the serial buffer input
        Serial.flush();
        
        // Return ACK with original frame
        for(i=0;i<6;i++){
            Serial.write(myMotorCommand[i]);
          }
    }
  }
  
  delay(250);
}

/**
 * @brief Get the serial input frame, wait for 6 bytes minimum include SOF and EOF
 * 
 * @param myMotorCommand 
 * @return char 
 */

  char get_serial_input_frame(unsigned char * myMotorCommand){
    if (Serial.available() >= 6) {
      for(i=0;i<6;i++){
        myMotorCommand[i] = Serial.read();
      }
      return(1);
    }
    else return(0);
  }