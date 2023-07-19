#include <SPI.h>
#include <mcp2515.h> //download zip file library https://github.com/autowp/arduino-mcp2515

//#define TEST_ID  0x03F2
#define TEST_ID  0x80000000

// Printf Variables
char msgString[128];

// CAN TX Variables
struct can_frame canMsgS;
unsigned long prevTX = 0;                   // Variable to store last execution time
const unsigned int invlTX = 1000;           // One second interval constant
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};// Generic CAN data to send

// CAN RX Variables
struct can_frame canMsgR;

#define CAN0_INT  2               // Set INT to pin 2                   
MCP2515 mcp2515(10);              // Set CS to pin 10

void setup() {
  //Setup the CAN_TX message
  canMsgS.can_id = TEST_ID;   canMsgS.can_dlc = 8; 
  canMsgS.data[0] = data[0];  canMsgS.data[1] = data[1];  canMsgS.data[2] = data[2];  canMsgS.data[3] = data[3]; 
  canMsgS.data[4] = data[4];  canMsgS.data[5] = data[5];  canMsgS.data[6] = data[6];  canMsgS.data[7] = data[7];
  
  Serial.begin(9600);
  
  SPI.begin();
  if( mcp2515.reset() == MCP2515 :: ERROR_OK ){
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    //mcp2515.setNormalMode();
    mcp2515.setLoopbackMode();
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else
    Serial.println("Error Initializing MCP2515...");
  
  pinMode(CAN0_INT, INPUT);                     // Configuring pin for /INT input
  Serial.println("MCP2515 Loopback ...");
}

void loop() {
  if(!digitalRead(CAN0_INT))               // If CAN0_INT pin is low, read receive buffer
  {
    mcp2515.readMessage(&canMsgR);
    
    Serial.println("");
    if((canMsgR.can_id & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX DLC: %1d  Data: ", (canMsgR.can_id & 0x1FFFFFFF), canMsgR.can_dlc);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX DLC: %1d  Data: ", canMsgR.can_id, canMsgR.can_dlc);
  
    Serial.print(msgString);
    if((canMsgR.can_id & 0x40000000) == 0x40000000){  // message is a remote request frame?
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
      } 
    else {
      for(byte i = 0; i<canMsgR.can_dlc; i++){
        sprintf(msgString, " 0x%.2X", canMsgR.data[i]);
        Serial.print(msgString);
      }
    }
  }
  if(millis() - prevTX >= invlTX){           // Send this at a one second interval. 
    prevTX = millis();
    Serial.print("  ");
    
    if( mcp2515.sendMessage(&canMsgS) == MCP2515 :: ERROR_OK )
      Serial.print("CAN Message Sent Successfully!");
    else
      Serial.print("Error Sending CAN Message...");
  }
}
