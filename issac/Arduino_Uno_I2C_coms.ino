/*--------------------------------------------------------------------------------------------------------------------
 * The code been developed for the Soft Touch Prosthetic Hand Project as part of the MEng Robotics ROCO504 Module at Plymouth University.
 * This code has been designed for the Arduino Uno microcontroller.
 * This file is subject to the terms and conditions defined in the file 'LICENSE.txt'.
 * Written by Isaac Chasteau
 --------------------------------------------------------------------------------------------------------------------*/

#include <Wire.h>

#define index_address 0x4
#define middle_address 0x5
#define ring_address 0x6
#define pinky_address 0x7
#define thumb_address 0x8

#define MCPOpen 20
#define proximalOpen 20

#define MCPClosed 70
#define proximalClosed 70

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void demo(void){
int allDelay = 3500;
int singleDelay = 2000;

  sendAllFingersPosition(MCPOpen, proximalOpen, 'e');//open all
  sendPosition(MCPOpen, proximalOpen, 'e', 'e', thumb_address);//close thumb
  delay(allDelay*2);
//  sendAllFingersPosition(MCPClosed, proximalClosed, 'e');//close all
//  sendPosition(MCPClosed, proximalClosed, 'e', 'e', thumb_address);//close thumb
//  delay(allDelay);
//  sendAllFingersPosition(MCPOpen, proximalOpen, 'e');//open all
//  sendPosition(MCPOpen, proximalOpen, 'e', 'e', thumb_address);//close thumb
//  delay(allDelay);
  
  sendPosition(MCPClosed, proximalClosed, 0, 'e', pinky_address);//close pinky
  delay(singleDelay);
  sendPosition(MCPClosed, proximalClosed, 0, 'e', ring_address);//close ring
  delay(singleDelay);
  sendPosition(MCPClosed, proximalClosed, 0, 'e', middle_address);//close middle
  delay(singleDelay);
  sendPosition(MCPClosed, proximalClosed, 0, 'e', index_address);//close index
  delay(singleDelay);
  sendPosition(MCPClosed, proximalClosed, 'e', 'e', thumb_address);//close thumb
  delay(singleDelay);
  
  sendPosition(MCPOpen, proximalOpen, 0, 'e', pinky_address);//open pinky
  delay(singleDelay);
  sendPosition(MCPOpen, proximalOpen, 0, 'e', ring_address);//open ring
  delay(singleDelay);
  sendPosition(MCPOpen, proximalOpen, 0, 'e', middle_address);//open middle
  delay(singleDelay);
  sendPosition(MCPOpen, proximalOpen, 0, 'e', index_address);//open index
  delay(singleDelay);
  sendPosition(MCPOpen, proximalOpen, 'e', 'e', thumb_address);//open thumb
  delay(singleDelay);
  
  sendPosition(MCPOpen, proximalClosed, 0, 'e', pinky_address);//close pinky proximal
  delay(singleDelay);
  sendPosition(MCPOpen, proximalClosed, 0, 'e', ring_address);//close ring proximal
  delay(singleDelay);
  sendPosition(MCPOpen, proximalClosed, 0, 'e', middle_address);//close middle proximal
  delay(singleDelay);
  sendPosition(MCPOpen, proximalClosed, 0, 'e', index_address);//close index proximal
  delay(singleDelay);
  sendPosition(MCPOpen, proximalClosed, 'e', 'e', thumb_address);//close thumb proximal
  delay(singleDelay);
  
  sendAllFingersPosition(MCPClosed, proximalClosed, 'e');//close all
  sendPosition(MCPOpen, proximalOpen, 'e', 'e', thumb_address);//close thumb
  delay(allDelay);
  sendAllFingersPosition(MCPClosed, proximalOpen, 'e');//open all proximal
  sendPosition(MCPClosed, proximalOpen, 'e', 'e', thumb_address);//open thumb proximal
  delay(allDelay);
  sendAllFingersPosition(MCPOpen, proximalOpen, 'e');//open hand
  sendPosition(MCPOpen, proximalOpen, 'e', 'e', thumb_address);//open thumb
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void sendAllFingersPosition(int MCP_pos, int proximal_pos, int co_contraction){
  char data[3];
  data[0] = MCP_pos;//MCP
  data[1] = proximal_pos;//Proximal
  data[2] = co_contraction;//Co-contraction
  data[3] = '\0';
  
  I2C_data(data, index_address);
  I2C_data(data, middle_address);
  I2C_data(data, ring_address);
  I2C_data(data, pinky_address);
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
//void test(void){
//  
//}
void sendPosition(int MCP_pos, int proximal_pos, int thumbServo_pos, int co_contraction, int address){
  char data[5];
  data[0] = MCP_pos;//MCP
  data[1] = proximal_pos;//Proximal
  data[2] = co_contraction;//Co-contraction
  if(address == thumb_address){
    data[3] = thumbServo_pos;
  }else{
    data[3] = '\0';
  }
  data[4] = '\0';
  
  I2C_data(data, address);
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void I2C_data(char chars[], int slave_address){
  Wire.beginTransmission(slave_address); // transmit to device #8
  Wire.write(chars);// sends byte
  Wire.endTransmission();// stop transmitting
  delay(1); 
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

//Send C%%% (10-100)
//Send Mdeg (0-90)
//Send Pdeg (0-90)
void I2CFingerControl(void){//address needs to be read from the input
  char I2cData[12];
  char numData[5];
  int i = 0;
  String data;
  int address = 0;
  int numberOfBytes = 7;
  char  numberVal3;
  
  if(Serial.available()){
    address = Serial.read();//read the first byte
  }

  if(address == 0){
    return;
  }else if(address == '1'){
    address = index_address;
    numberOfBytes = 7;
  }else if(address == '2'){
    address = middle_address;
    numberOfBytes = 7;
  }else if(address == '3'){
    address = ring_address;
    numberOfBytes = 7;
  }else if(address == '4'){
    address = pinky_address;
    numberOfBytes = 7;
  }else if(address == '5'){
    address = thumb_address;
    numberOfBytes = 10;
  }
  
  while(Serial.available()>0 && i < numberOfBytes)//reads the memaining bytes
  {
    char letter = Serial.read();
    data = data + letter;//forms a string from the bytes
    i++;
  }
  data.toCharArray(I2cData, numberOfBytes + 1);//change the string to a char array
  
  if(data.length() != numberOfBytes)return;
  char numberVal = 10 * (I2cData[0] - '0') + I2cData[1] - '0';
  char  numberVal1 = 10 * (I2cData[2] - '0') + I2cData[3] - '0';
  char  numberVal2 = 100 * (I2cData[4] - '0') + 10 * (I2cData[5] - '0') + I2cData[6] - '0';
  
  
  if(numberOfBytes == 10){
    numberVal3 = 100 * (I2cData[7] - '0') + 10 * (I2cData[8] - '0') + I2cData[9] - '0';
    if(numberVal3 == 0){
      numberVal3 = 'e';
    }
  }
  
  if(numberVal == 0){
    numberVal = 'e';
  }
  if(numberVal1 == 0){
    numberVal1 = 'e';
  }
  if(numberVal2 == 0){
    numberVal2 = 'e';
  }
  numData[0] = numberVal;
  numData[1] = numberVal1;
  numData[2] = numberVal2;
  
  if(numberOfBytes == 7){
    numData[3] = '\0';
  }else if(numberOfBytes == 10){
    numData[3] = numberVal3;
    numData[4] = '\0';
  }
  Serial.println((int)numData[0]);
  Serial.println((int)numData[1]); 
  Serial.println((int)numData[2]); 
  Serial.println((int)numData[3]); 
  Serial.println((int)numData[4]);   
  I2C_data(numData, address);
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void grip(int ms){
  //target pos = closed
  delay(ms);
  //stop motors/cocontraction
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void speedTest(int pause, int address){
  char data[4];
  data[0] = 10;//MCP
  data[1] = 5;//Proximal
  data[2] ='e';//Co-contraction
  data[3] = '\0';
  
  //while(1){
    //open
    data[0] = 5;//MCP
    data[1] = 5;//Proximal
    I2C_data(data, address);//probably needs to be 5 deg off
    delay(pause);
    //close
    data[0] = 85;//MCP
    data[1] = 85;//Proximal
    I2C_data(data, address);//probably needs to be 5 deg off
    delay(pause);
  //}
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void requestJointAngles(void){
  Wire.requestFrom(index_address, 2);
  Serial.print("Index: ");
  while (Wire.available()) { // slave may send less than requested
    int c = Wire.read(); // receive a byte as character
    Serial.print(c);
    Serial.print('\t');
  }
  Serial.println("");
  
  Wire.requestFrom(middle_address, 2);
  Serial.print("Middle: ");
  while (Wire.available()) { // slave may send less than requested
    int c = Wire.read(); // receive a byte as character
    Serial.print(c);
    Serial.print('\t');
  }
  Serial.println("");
  
  Wire.requestFrom(ring_address, 2);
  Serial.print("Ring: ");
  while (Wire.available()) { // slave may send less than requested
    int c = Wire.read(); // receive a byte as character
    Serial.print(c);
    Serial.print('\t');
  }
  Serial.println("");
  
  Wire.requestFrom(pinky_address, 2);
  Serial.print("Pinky: ");
  while (Wire.available()) { // slave may send less than requested
    int c = Wire.read(); // receive a byte as character
    Serial.print(c);
    Serial.print('\t');
  }
  Serial.println("");
  
  Wire.requestFrom(thumb_address, 2);
  Serial.print("Thumb: ");
  while (Wire.available()) { // slave may send less than requested
    int c = Wire.read(); // receive a byte as character
    Serial.print(c);
    Serial.print('\t');
  }
  Serial.println("");
  Serial.println("");
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

byte testVar= 0;
char incomingByte = 0;

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop(){
  //speedTest(5000, index_address);
  //demo();
  I2CFingerControl();
  requestJointAngles();
//  Wire.requestFrom(index_address, 2);    // request x bytes from slave device
//  while (Wire.available()) { // slave may send less than requested
//    int c = Wire.read(); // receive a byte as character
//    Serial.println(c);
//  }
//  Serial.println("");
  delay(100);
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
