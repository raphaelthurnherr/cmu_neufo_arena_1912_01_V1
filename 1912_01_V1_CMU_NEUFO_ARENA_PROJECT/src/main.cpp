#include <Arduino.h>
#include <U8x8lib.h>
#include "cmu_ws_1912_01_V1_board.h"
#include "cmu_ws_1921_01_V1_board.h"

#define SERIAL_DEBUG

#define vMax 3.33

#define maxMotorCourseStep 800

// Main board 1921 - IC7 digital output bit definition
#define GATE_A_ENABLE_BIT 6
#define GATE_B_ENABLE_BIT 5
#define GATE_C_ENABLE_BIT 4

// Main board 1921 - IC7 digital input bit definition
#define GATE_A_EOC_BIT    3
#define GATE_B_EOC_BIT    2
#define GATE_C_EOC_BIT    1

#define GATE_A_CMD_ON_OFF_BIT 8
#define GATE_A_CMD_CUSTOM_BIT 13
#define GATE_B_CMD_ON_OFF_BIT 12
#define GATE_B_CMD_CUSTOM_BIT 11
#define GATE_C_CMD_ON_OFF_BIT 10
#define GATE_C_CMD_CUSTOM_BIT 9

// Main board 1921 - IC3 analog input bit definition
#define POT_A_BIT         2
#define POT_B_BIT         1
#define POT_C_BIT         0

#define CLOSE  0
#define OPEN   1
#define CUSTOM 2


U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

board_1921_01_V01 arduino_1921_board;
board_1912_01_V01 motor_1912_board;

char correction;
char motor_A_actual_position_percent, motor_B_actual_position_percent, motor_C_actual_position_percent = 50;
int gate_A_custom_setpoint, gate_B_custom_setpoint, gate_C_custom_setpoint;
char gate_A_target_state, gate_B_target_state, gate_C_target_state;
char gate_A_is_moving, gate_B_is_moving, gate_C_is_moving;
char gate_A_setpoint, gate_B_setpoint, gate_C_setpoint;

// Variable for end-of-courses switchs states
char gate_A_EOC_state, gate_B_EOC_state, gate_C_EOC_state;
// Functions déclaration
int open_gate_and_calibration(unsigned char motor_number, unsigned char gate_eoc_bit);
char get_gate_command_from_inputs(unsigned char onOff_signal_bit, unsigned char custom_signal_bit);
void get_gates_end_of_courses_state(char * gateAeoc, char * gateBeoc, char * gateCeoc);
void check_motors_end_of_course(void);
void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:


  // Main board 1921 initialization
  arduino_1921_board.begin();
  // Motor board 1912 initialization
  motor_1912_board.begin();
  // OLED display initialization
  u8x8.begin();
  u8x8.setFont(u8x8_font_7x14B_1x2_f);
  u8x8.drawString(0, 0, "MOUSE ARENA V1");
  u8x8.drawString(0, 2, "Calibration...");

  // Enable the motors power H-BRIDGES
  arduino_1921_board.setDigitalOutput(GATE_A_ENABLE_BIT,1);     // Gate C enable
  arduino_1921_board.setDigitalOutput(GATE_B_ENABLE_BIT,1);     // Gate B enable
  arduino_1921_board.setDigitalOutput(GATE_A_ENABLE_BIT,1);     // Gate A enable

  // Start calibration for the gates end-of-courses 
  open_gate_and_calibration(MOTOR_A, GATE_A_EOC_BIT);
  delay(100);
  open_gate_and_calibration(MOTOR_B, GATE_B_EOC_BIT);
  delay(100);
  open_gate_and_calibration(MOTOR_C, GATE_C_EOC_BIT);
  delay(100);

  // Display the main user HID for gate position
  u8x8.clear();
  u8x8.drawString(0, 0, "Gate: A   B   C");
  u8x8.drawString(0, 2, "Pos:");
}


void loop() {
  // GET THE STATE OF ALL THE END OF COURSES SWITCH
  get_gates_end_of_courses_state(&gate_A_EOC_state, &gate_B_EOC_state, &gate_C_EOC_state);

  gate_A_is_moving = motor_1912_board.getStepperState(MOTOR_A);
  gate_B_is_moving = motor_1912_board.getStepperState(MOTOR_B);
  gate_C_is_moving = motor_1912_board.getStepperState(MOTOR_C);

  // Stop the motors when the end-of-course switch is activate and reset the motor position variable
  check_motors_end_of_course();

  // Get the POTENTIOMETER position
  gate_A_custom_setpoint = ((arduino_1921_board.getVoltage(POT_A_BIT) / 10)  / vMax);
  gate_B_custom_setpoint = ((arduino_1921_board.getVoltage(POT_B_BIT) / 10)  / vMax);
  gate_C_custom_setpoint = ((arduino_1921_board.getVoltage(POT_C_BIT) / 10)  / vMax);

  // Print the current setpoint for the gates
  u8x8.setCursor(5, 2);
  u8x8.print(gate_A_custom_setpoint);
  u8x8.print("% ");
  u8x8.setCursor(9, 2);
  u8x8.print(gate_B_custom_setpoint);
  u8x8.print("% ");
  u8x8.setCursor(13, 2);
  u8x8.print(gate_C_custom_setpoint);
  u8x8.print("%");


//*********************************************

  // Get gate command (OPEN/CLOSE OR CUSTOM) and apply the new setpoint
  gate_A_target_state = get_gate_command_from_inputs(GATE_A_CMD_ON_OFF_BIT, GATE_A_CMD_CUSTOM_BIT);
    switch(gate_A_target_state){
      case OPEN : gate_A_setpoint = 100 ; break;
      case CLOSE : gate_A_setpoint = 0 ; break;
      case CUSTOM : gate_A_setpoint = gate_A_custom_setpoint ; break;
      default: gate_A_setpoint = 0 ;
  }

  // Get gate command (OPEN/CLOSE OR CUSTOM) and apply the new setpoint
  gate_B_target_state = get_gate_command_from_inputs(GATE_B_CMD_ON_OFF_BIT, GATE_B_CMD_CUSTOM_BIT);
    switch(gate_B_target_state){
      case OPEN : gate_B_setpoint = 100 ; break;
      case CLOSE : gate_B_setpoint = 0 ; break;
      case CUSTOM : gate_B_setpoint = gate_B_custom_setpoint ; break;
      default: gate_B_setpoint = 0 ;
  }

  // Get gate command (OPEN/CLOSE OR CUSTOM) and apply the new setpoint
  gate_C_target_state = get_gate_command_from_inputs(GATE_C_CMD_ON_OFF_BIT, GATE_C_CMD_CUSTOM_BIT);
    switch(gate_C_target_state){
      case OPEN : gate_C_setpoint = 100 ; break;
      case CLOSE : gate_C_setpoint = 0 ; break;
      case CUSTOM : gate_C_setpoint = gate_C_custom_setpoint ; break;
      default: gate_C_setpoint = 0 ;
  }

//*********************************************
  if(motor_A_actual_position_percent != gate_A_setpoint){

    Serial.print("Moving A to new setpoint: ");
    Serial.println(gate_A_setpoint, DEC);

    if(gate_A_setpoint >=100){
      correction = 120;
      Serial.println("Finding for A EOC...");
    }
    else
      correction = gate_A_setpoint - motor_A_actual_position_percent;

    if(!gate_A_is_moving){
      motor_1912_board.stepperRotation(MOTOR_A, 95, correction * -6);
      motor_A_actual_position_percent = gate_A_setpoint;  
    } else {
      Serial.println("Motor A is actually moving !");
    }
  }

    if(motor_B_actual_position_percent != gate_B_setpoint){

    Serial.print("Moving B to new setpoint: ");
    Serial.println(gate_B_setpoint, DEC);

    if(gate_B_setpoint >=100){
      correction = 120;
      Serial.println("Finding for B EOC...");
    }
    else
      correction = gate_B_setpoint - motor_B_actual_position_percent;

    if(!gate_B_is_moving){
      motor_1912_board.stepperRotation(MOTOR_B, 95, correction * -6);
      motor_B_actual_position_percent = gate_B_setpoint;  
    } else {
      Serial.println("Motor B is actually moving !");
    }
  }

    if(motor_C_actual_position_percent != gate_C_setpoint){

    Serial.print("Moving C to new setpoint: ");
    Serial.println(gate_C_setpoint, DEC);

    if(gate_C_setpoint >=100){
      correction = 120;
      Serial.println("Finding for C EOC...");
    }
    else
      correction = gate_C_setpoint - motor_C_actual_position_percent;

    if(!gate_C_is_moving){
      motor_1912_board.stepperRotation(MOTOR_C, 95, correction * -6);
      motor_C_actual_position_percent = gate_C_setpoint;  
    } else {
      Serial.println("Motor C is actually moving !");
    }
  }
  delay(20);
  }


/**
 * @brief START THE SPECIFED MOTOR FOR OPEN THE DOOR TO THE END-OF-COURSE SIGNAL
 * 
 * @param motor_number 
 * @param gate_eoc_bit 
 * @return int 
 */

  int open_gate_and_calibration(unsigned char motor_number, unsigned char gate_eoc_bit){
  unsigned char gate_EOC = 0;

  #ifdef SERIAL_DEBUG
  Serial.print("START CALIBRATION FOR MOTOR ");
  Serial.print(motor_number);
  Serial.print(" ...");
  #endif

  motor_1912_board.stepperRotation(motor_number, 80, maxMotorCourseStep*-1);
  while(!gate_EOC)
  {
    gate_EOC = arduino_1921_board.getDigitalInput(gate_eoc_bit);
    delay(20);
  }
  motor_1912_board.stepperRotation(motor_number, 0, 1);

  #ifdef SERIAL_DEBUG
  Serial.println(" END");
  #endif

  // return the actual position [%]  (0%=close, 100%=open)
  return 100;
}




/**
 * @brief Get the gate command from inputs object
 * 
 * @param onOff_signal_bit 
 * @param custom_signal_bit 
 * @return char 
 */

char get_gate_command_from_inputs(unsigned char onOff_signal_bit, unsigned char custom_signal_bit){
  unsigned char gate_command;

// Check if custom position is requiered
 if(arduino_1921_board.getDigitalInput(custom_signal_bit)){
   gate_command = CUSTOM;
 }else
 {
    if(arduino_1921_board.getDigitalInput(onOff_signal_bit))
      gate_command = OPEN;
    else 
      gate_command = CLOSE;
 }
 
  return gate_command;
}

/**
 * @brief Get the gates end of courses state object
 * 
 * @param gateAeoc 
 * @param gateBeoc 
 * @param gateCeoc 
 */
void get_gates_end_of_courses_state(char * gateAeoc, char * gateBeoc, char * gateCeoc){
  *gateAeoc = arduino_1921_board.getDigitalInput(GATE_A_EOC_BIT);
  *gateBeoc = arduino_1921_board.getDigitalInput(GATE_B_EOC_BIT);
  *gateCeoc = arduino_1921_board.getDigitalInput(GATE_C_EOC_BIT);
}

  void check_motors_end_of_course(void){
    static char gate_A_EOC_one_shot_detect, gate_B_EOC_one_shot_detect, gate_C_EOC_one_shot_detect=0;
    if(gate_A_EOC_state){
      if(!gate_A_EOC_one_shot_detect){
        motor_1912_board.stepperRotation(MOTOR_A, 0, 1);
        Serial.println("Reset motor A position to FULL OPEN");
        gate_A_EOC_one_shot_detect = 1;
        motor_A_actual_position_percent = 100;
      }
    }else gate_A_EOC_one_shot_detect = 0;

    if(gate_B_EOC_state){
      if(!gate_B_EOC_one_shot_detect){
        motor_1912_board.stepperRotation(MOTOR_B, 0, 1);
        Serial.println("Reset motor B position to FULL OPEN");
        gate_B_EOC_one_shot_detect = 1;
        motor_B_actual_position_percent = 100;
      }
    }else gate_B_EOC_one_shot_detect = 0;

    if(gate_C_EOC_state){
      if(!gate_C_EOC_one_shot_detect){
        motor_1912_board.stepperRotation(MOTOR_C, 0, 1);
        Serial.println("Reset motor C position to FULL OPEN");
        gate_C_EOC_one_shot_detect = 1;
        motor_C_actual_position_percent = 100;
      }
    }else gate_C_EOC_one_shot_detect = 0;
  }