#include <SPI.h>
#include <mcp2515.h> //download zip file library https://github.com/autowp/arduino-mcp2515
#include <stdint.h>
#include<avr/wdt.h> /* Header for watchdog timers in AVR */

/*For testing, you can use the following string
70 01 00 00 08 00 00 00 10 20 30 40 50 60 70 80 FA FB FC
paste in Hex in  Commix 1.4 (or your preferred Serial console ) and send it as HEX
*/ 

/*Serial Ring Buffer*/
#define SIZE_SERIAL_BUFFER 256/*size of serial ring buffer*/
#define CAN_RESET_TIMEOUT 5000/*reset can comunication after CAN_RESET_TIMEOUT millis without mesagess*/
#define TOGGLE_LIGHTS 0X01

#define EMERGENCY_LIGHTS_CAN_ID 0X170

uint8_t end_of_frame_patern[2]={0xFA,0XFB};

/*Declarate struct of serial ring buffer*/
typedef struct{
  uint8_t index_data_received;
  uint8_t index_data_processced;
  uint8_t buffer[SIZE_SERIAL_BUFFER];/*The received data is stored in this array*/
  uint8_t packets_received;/*when a packet is received this variable is increased*/
}strserial;
volatile strserial serial;

/*Ring buffer Init*/
void ring_buffer_init(void){
  serial.index_data_processced = 0x00;
  serial.index_data_received = 0x00;
  serial.packets_received = 0; 
}

#define CLUSTER_ID  0x03F2 //Unique identifier for the cluster in this project

//Time variable
unsigned long time;

// Printf Variables
char msgString[128];

// CAN TX Variables
struct can_frame canMsgT;
void * voidptrT = &canMsgT;//Cast canMsgT to void pointer
char * ptrT = voidptrT;//Cast canMsgT to char pointer

// CAN RX Variables
struct can_frame canMsgR;
void * voidptrR = &canMsgR;//Cast canMsgR to void pointer
char * ptrR = voidptrR;//Cast canMsgR to char pointer

#define CAN0_INT  2               // Set INT to pin 2                   
uint8_t flag_CAN_Message_received = 0;
MCP2515 mcp2515(10);              // Set CS to pin 10

void mcp2515_init(void){
  SPI.begin();
  if( mcp2515.reset() == MCP2515 :: ERROR_OK ){
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    //mcp2515.setLoopbackMode();
    //Serial.println("MCP2515 Initialized Successfully!");
  }
  else
    ;
    //Serial.println("Error Initializing MCP2515...");
}

void setup() {
  pinMode(CAN0_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN0_INT), CANinterrupt, FALLING);
  Serial.begin(115200);
  //Initialize Ring Buffer
  ring_buffer_init();
  
  wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */
  delay(3000);  /* Done so that the Arduino doesn't keep resetting infinitely in case of wrong configuration */
  wdt_enable(WDTO_2S);  /* Enable the watchdog with a timeout of 2 seconds*/

  mcp2515_init();
  pinMode(CAN0_INT, INPUT);// Configuring pin for /INT input
  time = millis();
}

void loop() {
  if(millis()-time >= CAN_RESET_TIMEOUT)
    mcp2515_init();

  if(flag_CAN_Message_received)//If CAN0_INT pin is low, read receive CAN buffer
  {
    flag_CAN_Message_received = 0;
    if( mcp2515.readMessage(&canMsgR) != MCP2515 :: ERROR_OK ){//Receive CAN buffer
      //Serial.print("Error receiving CAN Message...");
      mcp2515_init();
    }

    /*********TRANSMIT CAN MESSAGE BY UART**********/
      //Serial send START patern
      Serial.write(0xff);
      Serial.write(0xfc);
      Serial.write(0xfd);

      //serial send can structure
        Serial.write(ptrR[0]);//send CAN_ID low to uart
        Serial.write(ptrR[1]);//send CAN_ID hi to uart
        Serial.write(ptrR[4]);//send CAN_DLC to uart
        for(uint8_t i = 0 ; i < ptrR[4] ; i++){//send CAN_DATA to uart  (CAN_DLC BYTES)
          Serial.write(ptrR[8+i]);
        }

      // serial send end of frame patern
      Serial.write(end_of_frame_patern[0]);
      Serial.write(end_of_frame_patern[1]);
      /*********END TRANSMIT CAN MESSAGE BY UART**********/

      wdt_reset();  /* Reset the watchdog */
  
    time = millis();
  }

  //serial packet received
  if (serial.packets_received) {
    //cast serial to can structure

    if(serial.buffer[++serial.index_data_processced]==TOGGLE_LIGHTS)//Solicita el cambio de estado de las luces de emergencia
    {
      canMsgT.can_id=EMERGENCY_LIGHTS_CAN_ID;
      canMsgT.can_dlc=0X01;
      //Send can message  
      if( mcp2515.sendMessage(&canMsgT) != MCP2515 :: ERROR_OK ){
        //Serial.print("Error Sending CAN Message...");
        mcp2515_init();
      }

    }
    serial.index_data_processced+=2;
    serial.packets_received--;//serial packet proseced 
    if(! serial.packets_received ){
      serial.index_data_processced = serial.index_data_received;
    }
  }
}

void CANinterrupt(){
  flag_CAN_Message_received = 1;

}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    serial.buffer[++serial.index_data_received] = (uint8_t)Serial.read();
    //Detect end of frame
      if(serial.buffer[serial.index_data_received ] == end_of_frame_patern[1])
        if(serial.buffer[serial.index_data_received - 1] == end_of_frame_patern[0])         
          serial.packets_received++;//End of frame detected, increased counter
  }
}