#include <SPI.h>
#include <mcp2515.h> //download zip file library https://github.com/autowp/arduino-mcp2515
#include <stdint.h>


/*Serial Ring Buffer*/
#define SIZE_SERIAL_BUFFER 256/*size of serial ring buffer*/
uint8_t end_of_frame_patern[3]={0xFA,0XFB,0XFC};

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
  serial.index_data_processced = 0xff;
  serial.index_data_received = 0x00;
  serial.packets_received = 0; 
}

#define CLUSTER_ID  0x03F2 //Unique identifier for the cluster in this project

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
MCP2515 mcp2515(10);              // Set CS to pin 10

void setup() {
  Serial.begin(115200);
  //Initialize Ring Buffer
  ring_buffer_init();

  SPI.begin();
  if( mcp2515.reset() == MCP2515 :: ERROR_OK ){
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    //mcp2515.setLoopbackMode();
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else
    Serial.println("Error Initializing MCP2515...");
  pinMode(CAN0_INT, INPUT);// Configuring pin for /INT input
}

void loop() {
  if(!digitalRead(CAN0_INT))//If CAN0_INT pin is low, read receive CAN buffer
  {
    mcp2515.readMessage(&canMsgR);//Receive CAN buffer
    if (canMsgR.can_id == CLUSTER_ID){
      //serial send can structure
      for(uint8_t i = 0 ; i < sizeof(canMsgR); i++)
        Serial.write(ptrR[i]);

      // serial send end of frame patern
      Serial.write(end_of_frame_patern[0]);
      Serial.write(end_of_frame_patern[1]);
      Serial.write(end_of_frame_patern[2]);
    } 
  }

  //serial packet received
  if (serial.packets_received) {
    //cast serial to can structure
    for(uint8_t i = 0 ; i < sizeof(canMsgR); i++)
      ptrT[i] = serial.buffer[++serial.index_data_processced];  
    serial.index_data_processced+=3;
    
    //Send can message  
    if( mcp2515.sendMessage(&canMsgT) != MCP2515 :: ERROR_OK )
      Serial.print("Error Sending CAN Message...");
    
    serial.packets_received--;//serial packet proseced 
  }
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
    if(serial.buffer[serial.index_data_received] == end_of_frame_patern[2])
      if(serial.buffer[serial.index_data_received - 1] == end_of_frame_patern[1])
        if(serial.buffer[serial.index_data_received - 2] == end_of_frame_patern[0])
          serial.packets_received++;//End of frame detected, increased counter
  }
}