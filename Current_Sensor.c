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

// Global Variables and Constants

#define GLV_current_sensor_MIN 0
#define GLV_current_sensor_MAX 1023
#define GLV_current_sensor_PIN A7 // signal for LEM_DHAB
byte GLV_current_sensor_Data[1] = {0x00};

// Function Declarations

void GLV_current_sensor_status();


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
  
}

/********** Main Loop ***************/

void loop() {
  /* Anything you want run forever, put inside loop.*/

  unsigned long canId = CAN.getCanId();

  // output functions


  // input functions
  GLV_current_sensor_status();


  can_delay_cycle = millis() - can_timeold;
  if (can_delay_cycle >= CAN_MSG_DELAY) message_cycle();
}

/********** Function Implementations ***************/
/* Below, put any of your function implementations*/

  // Checks fuel Pump Status
void GLV_current_sensor_status()
{
  // Reads voltage output from sensor
  int GLV_current_read = analogRead(GLV_current_sensor_PIN);
  int GLV_correction_read = GLV_current_read - GLV_current_sensor_MIN;

  
  float GLV_current_sensor_voltage = (5.0 * GLV_correction_read) / 1023;
  float GLV_current_sensor_current = (GLV_current_sensor_voltage - 2.5)*(1/0.0667);

  //GLV_current_sensor_Data[0] = GLV_current_sensor_current & 0xff;
  
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
      send_CAN_msg(0x55, 0, 1, GLV_current_sensor_Data[0]);
      message_num++;
      break;
   

    default:
      //reset message number if issue is encountered.
      message_num = 0;
  }

  can_timeold = millis();
}
