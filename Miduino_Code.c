#include <SPI.h>
#include "mcp2515_can.h"
#include "mcp2515_can_dfs.h"


// -> CAN message vars

#define SPI_CS_PIN 9 // CAN specific pins
#define CAN_INT_PIN 2 // CAN interrupt pin
#define  CAN_MSG_DELAY 100
unsigned long can_timeold;
int can_delay_cycle;
int message_num = 0;

// Global Variables and 

#define GLV_current_sensor_MIN 0
#define GLV_current_sensor_MAX 1023
#define GLV_current_sensor_PIN A7 // signal for LEM_DHAB
byte GLV_current_sensor_Data[1] = {0x00};

#define Fuel_MIN 0
#define Fuel_MAX 1023
#define Fuel_PIN ## // signal for Fuel_Pump
byte Fuel_Pump_OK[1] = {0x00};

#define Fan_MIN 0
#define Fan_MAX 1023
#define Fan_PIN ## // signal for Fan
byte Fan_OK[1] = {0x00};

#define Brake_MIN 0
#define Brake_MAX 1023
#define Brake_PIN ## // signal for Brake
byte Brake_OK[1] = {0x00};

#define Start_MIN 0
#define Start_MAX 1
#define Start_PIN ## // signal for Start
byte Start_OK[1] = {0x00};

#define Shift_Up_MIN 0
#define Shift_Up_MAX 1
#define Shift_Up_PIN ## // signal for shift up
byte Shift_Up[1] = {0x00};

#define Shift_Down_MIN 0
#define Shift_Down_MAX 1
#define Shift_Down_PIN ## // signal for shift up
byte Shift_Down[1] = {0x00};

#define Throttle_MIN 0
#define Throttle_MAX 1023
#define Throttle_output_PIN # // Throttle
byte Throttle[1] = {0x00};

// GetCAN IDs for functions that produce an output
unsigned char Acellerator_percentage;
can.readMsgBuf( 1, &Acellerator_percentage);
#define Shift_Up_id = #x##
#define Shift_Down_id = #x##




// Function Declarations

// Input Functions
void Fuel_Pump_Check();
void Fan_Check();
void Brake_Check();
void Start_Check();
void GLV_current_sensor_status();

// Output Functions
void Shift_Up();
void Shift_Down();
void Throttle();



/********** Setup/Initialization ***************/

mcp2515_can CAN(SPI_CS_PIN); // set CS to pin 9

void setup() {
  /* Anything that might need to be initialized like setting the
     pinmode to input or output can be put inside setup.*/

  // set the speed of the serial port
  Serial.begin(115200);

  init_CAN();
  can_timeold = 0;

  pinMode(GLV_current_sensor_PIN, INPUT);
  pinMode(Fuel_Pin, INPUT);
  pinMode(Fan_Pin, INPUT);
  pinMode(Brake_Pin, INPUT);
  pinMode(Start_Pin, INPUT);
  pinMode(Shift_Up_Pin, OUTPUT);
  pinMode(Shift_Down_Pin, OUTPUT);
  pinMode(Throttle_Pin, OUTPUT);
  
}

/********** Main Loop ***************/

void loop() {
  /* Anything you want run forever, put inside loop.*/

  unsigned long canId = CAN.getCanId();

  // output functions
  switch(canId) {
     case accelerator_Percentage:
          throttle();
          break;
     
     case Shift_Up_id:
          Shift_up();
          break;

     case Shift_Down_id:
          Shift_Down();
          break;

     default:
          break;

  // input 
  GLV_current_sensor_status();
  Fuel_Pump_Check();
  Fan_Check();
  Brake_Check();
  Start_Check();

  can_delay_cycle = millis() - can_timeold;
  if (can_delay_cycle >= CAN_MSG_DELAY) message_cycle();
}

/********** Function Implementations ***************/
/* Below, put any of your function implementations*/

  // Checks fuel Pump Status
GLV_current_sensor_status();


  can_delay_cycle = millis() - can_timeold;
  if (can_delay_cycle >= CAN_MSG_DELAY) message_cycle();
}

/********** Function Implementations ***************/
/* Below, put any of your function implementations*/

  
void GLV_current_sensor_status()
{
  // Reads voltage output from sensor
  int GLV_current_read = analogRead(GLV_current_sensor_PIN);
  int GLV_correction_read = GLV_current_read - GLV_current_sensor_MIN;

  //Converts voltage read from sensor and calculates current going through sensor
  float GLV_current_sensor_voltage = (5.0 * GLV_correction_read) / 1023;
  float GLV_current_sensor_current = (GLV_current_sensor_voltage - 2.5)*(1/0.0667);

  //GLV_current_sensor_Data[0] = GLV_current_sensor_current & 0xff;
  
}

// Checks fuel Pump Status
Fuel_Pump_Check();
{
  int Fuel_Read = analogRead(Fuel_PIN)

  int Fuel_Corr = Fuel_Read - Fuel_min

  int Fuel_Percent = (100 * Fuel_Corr) / Fuel_Max;
  Fuel_Data[0] = Fuel_Percent & 0xff;
}

  // Checks fan status, store value Fan_OK[0] = {#x##}  
Fan_Check();
{
  int Fan_Read = analogRead(Fan_PIN)

  int Fan_Corr = Fan_Read - Fan_min

  int Fan_Percent = (100 * Fan_Corr) / Fan_Max;
  Fan_Data[0] = Fan_Percent & 0xff;
}

  // Checks brake status, store value Brake_OK[0] = {#x##}
Brake_Check();
{
  int Brake_Read = analogRead(Brake_PIN)

  int Brake_Corr = Brake_Read - Brake_min

  int Brake_Percent = (100 * Brake_Corr) / Brake_Max;
  Brake_Data[0] = Brake_Percent & 0xff;
}

  // Checks start status, store value Start_OK[0] = {#x##}
Start_Check();
{
  int Start_Read = analogRead(Start_PIN)

  int Start_Corr = Start_Read - Start_min

  int Start_Percent = (100 * Start_Corr) / Start_Max;
  Start_Data[0] = Start_Percent & 0xff;
}

  // Checks if shift up button is pressed
Shift_Up();
{
  int Shift_Up_Read = digitalRead(Shift_Up_PIN)
}

  // Checks if shift down button is pressed
Shift_Down();
{
  int Shift_Down_Read = digitalRead(Shift_Down_PIN)
}

  // Checks accelerator position outputs to throttle, sends PWM function through Throttle pin
Throttle();
{ 
  
  int acellerator_position = ((accelerator_id / 100.0) * 255);
  analogwrite(Throttle, acellerator_id);

}

/* 
  Function: Initialize MCP2515 (CAN system) running with a 
            baudrate of 500kb/s. Code will not continue until 
            initialization is OK.
*/
void init_CAN()
{
  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN init fail, retry...");
    delay(100);
  }
  Serial.println("CAN init ok!");
}

/* 
  Function: Sending a CAN message over the network.
  Params:   unsigned long id - The message ID.
            byte ext - The extension code for the message. typically 0.
            byte len - The message length. (max 8 bytes)
            const byte * msg_buf - The message byte array to be transmitted over CAN.
*/
void send_CAN_msg(unsigned long id, byte ext, byte len, const byte * msg_buf)
{
  byte send_status = CAN.sendMsgBuf(id, ext, len, msg_buf);

  if (send_status == MCP2515_OK) Serial.println("Message Sent Successfully!");
  else Serial.println("message Error...");
}

/*
  Function: The message_cycle() function is used to perpetually cycle through and
            send messages from the various sensors connected to the front canduino.
            Each global data array for the sensors is constantly updated from the main
            loop from which message_cycle() takes the data array and transmits the 
            message over the CAN network. 
  Params:   global sensor data buffers - The global variables are used to as containers
            to allow for indefinite updating of the sensor values. The fuction then reads
            those values and transmits them.
  Return:   NA
  Pre-conditions: CAN must first be initialized.
*/
void message_cycle()
{
  switch (message_num)
  {
    case 0:
      send_CAN_msg(0x55, 0, 1, Fuel_Pump_OK);
      message_num++;
      break;
    
    case 1:
      send_CAN_msg(0x66, 0, 1, Fan_OK);
      message_num++;
      break;
    
    case 2:
      send_CAN_msg(0x77, 0, 1, Brake_OK);
      message_num++;
      break;

    case 3:
      send_CAN_msg(0x88, 0, 1, Start_OK);
      message_num = ++;
      break;
    
    case 4;
      send_CAN_msg(0x88, 0, 1, Shift_Up);
      message_num = 0;
      break;
    
    case 5;
      send_CAN_msg(0x88, 0, 1, Shift_Down);
      message_num = ++;
      break;

    case 6;
    end_CAN_msg(0x88, 0, 1, Throttle);
      message_num = ++;
      break;

    default:
      //reset message number if issue is encountered.
      message_num = 0;
  }

  can_timeold = millis();
}
