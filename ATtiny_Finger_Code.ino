/*--------------------------------------------------------------------------------------------------------------------
 * The code been developed for the Soft Touch Prosthetic Hand Project as part of the MEng Robotics ROCO504 Module at Plymouth University.
 * This code has been designed for the Atmel ATtiny84 microcontroller running at 8MHz.
 * This file is subject to the terms and conditions defined in the file 'LICENSE.txt'.
 * Written by Isaac Chasteau
 --------------------------------------------------------------------------------------------------------------------*/

#include <TinyWireS.h>
#include <ctype.h>

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

/*--------------------------------------------------------------------------------------------------------------------
 * ATtiny84 I2C addresses
 * Index finger 0x4
 * Middle finger 0x5
 * Ring finger 0x6
 * Pinky finger 0x7
 * Thumb 0x8
 --------------------------------------------------------------------------------------------------------------------*/
 
#define I2C_SLAVE_ADDRESS 0x4// the 7-bit address (remember to change this when adapting this example)

//Defining the H-Bridge pins
#define IA_MCP PB2
#define IB_MCP PB0
#define IA_Proximal PA3//physical pin 6 (PWM)
#define IB_Proximal PB1
#define IA_Tension PA5//Physical pin 8
#define IB_Tension PA3//Physical pin 10(Digital)

//Defining the potentiometer pins
#define Proximal_pin PA2
#define MCP_pin PA1

//Defining PPAP parameters
#define Kp 18
#define Ki 0.45
#define Kd 500//My K/D you scrub

#define Kp1 25
#define Ki1 0.55
#define Kd1 600

#define integralUB 50
#define integralLB -50

#define minPWMpos 25
#define minPWMneg -30

#define minPWMpos1 35
#define minPWMneg1 -40

#define minMCPAngle 0
#define maxMCPAngle 90

#define minProximalAngle 0
#define maxProximalAngle 90

#define minCC 0
#define maxCC 102

int proximal_angle = 0;
int MCP_angle = 0;

//int previousTime = 0;
double previous_error = 0;
double integral = 0;
double derivative = 0;
double dt = 10;

//int previousTime1 = 0;
double previous_error1 = 0;
double integral1 = 0;
double derivative1 = 0;
double dt1 = 10;

int MCP_setPoint = 45;
int proximal_setPoint = 45;
int co_contraction = 0;

bool PID_cc_flag = false;
bool grip_flag = false;
bool PID_flag = true;
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

volatile uint8_t jointFeedback[] =
{
    0x0, 
    0x0, 
};

volatile uint8_t fingerVariables[] = //Holds the MCP and Proximal 
{
    0x1F, //MCP angle
    0x1F, //Proximal angle
    0x00, //Co-Contraction
};

volatile uint8_t bufferArray[] = //buffer 
{
    0x1F, //MCP angle
    0x1F, //Proximal angle
    0x00, //Co-Contraction
};
//const byte reg_size = sizeof(i2c_regs);
const byte fingerVariables_reg_size = sizeof(fingerVariables);
const byte two_reg_position_reg_size = sizeof(jointFeedback);
volatile byte reg_position = 0;
volatile byte three_reg_position = 0;
volatile byte two_reg_position = 0;
volatile byte reg_pos = 0;

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void updateJointValues(void){
  proximal_angle = analogRead(Proximal_pin);//readADC(2);//analogRead(Proximal_pin);
  MCP_angle = analogRead(MCP_pin);//readADC(1);//analogRead(MCP_pin);
  if(I2C_SLAVE_ADDRESS == 0x4){//Index finger
    proximal_angle = map(proximal_angle, 400, 860, 0, 90);
    MCP_angle = map(MCP_angle, 550, 840, 0, 90);
  }else if(I2C_SLAVE_ADDRESS == 0x5){//Middle finger
    proximal_angle = map(proximal_angle, 516, 988, 0, 90);
    MCP_angle = map(MCP_angle, 576, 1020, 0, 90);
  }else if(I2C_SLAVE_ADDRESS == 0x6){//Ring finger
    proximal_angle = map(proximal_angle, 590, 1020, 0, 90);
    MCP_angle = map(MCP_angle, 340, 804, 0, 90);
  }else if(I2C_SLAVE_ADDRESS == 0x7){//Pinky finger
    proximal_angle = map(proximal_angle, 330, 744, 0, 90);
    MCP_angle = map(MCP_angle, 550, 950, 0, 90);
  }else if(I2C_SLAVE_ADDRESS == 0x8){//Thumb
    proximal_angle = map(proximal_angle, 390, 820, 0, 90);
    MCP_angle = map(MCP_angle, 410, 840, 0, 90);
  }
  jointFeedback[0] = MCP_angle;
  jointFeedback[1] = proximal_angle;
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void requestEvent()
{  
  TinyWireS.send(jointFeedback[two_reg_position]);
  two_reg_position++;
  if (two_reg_position >= two_reg_position_reg_size)
  {
    two_reg_position = 0;
  }
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void receiveEvent(uint8_t howMany)
{
  if (howMany < 1)return;// Sanity-check
  if (howMany > TWI_RX_BUFFER_SIZE) return;
  if (!howMany) return; // This write was only to set the buffer for next read
  if (howMany != 3)return;//if it doesn't receive 3 bytes
  while(howMany--)
  {
    bufferArray[reg_pos] = TinyWireS.receive();//read in the data and stores it.
    if(bufferArray[reg_pos] == 101){ 
      bufferArray[reg_pos] = 0;
    }
    reg_pos++;
    if (reg_pos >= fingerVariables_reg_size)
    {
      reg_pos = 0;
      break;
    }
  }
  //Verifies the data is in the correct ranges
  if(bufferArray[0] < minMCPAngle || bufferArray[0] > maxMCPAngle )return;
  if(bufferArray[1] < minProximalAngle || bufferArray[1] > maxProximalAngle )return;
  if(bufferArray[2] < minCC || bufferArray[2] > maxCC )return;
  
  for(int i = 0; i < fingerVariables_reg_size; i++){//Sets the fingerVariable array to the bufferArray values
    fingerVariables[i] = bufferArray[i];
  }
  update_setpoints();
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void update_setpoints(void){
  MCP_setPoint = fingerVariables[0];
  proximal_setPoint = fingerVariables[1];
  co_contraction = fingerVariables[2];

  if(co_contraction == 0){
    PID_flag = true;
    PID_cc_flag = false;
    grip_flag = false;
  }else if(co_contraction > 0 && co_contraction <= 100){
    PID_flag = false;
    PID_cc_flag = true;
    grip_flag = false;
  }else if(co_contraction == 102){
    PID_flag = false;
    PID_cc_flag = true;
    grip_flag = true;
  }
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void motorDriver(int pwm, int motorNum){
  if(motorNum == 0){
    if (pwm >= 0){//If PWM is positive
      analogWrite(IA_MCP, pwm);
      PORTB &= (~(1<<IB_MCP));
    }else if(pwm < 0){//If PWM in negative 
      PORTB |= (1<<IB_MCP);//IB is set high
      analogWrite(IA_MCP, 255+pwm);//IA is a PWM of 255 minus the negative PWM
    }
  }else if(motorNum == 1){
    if (pwm >= 0){//If PWM is positive
      analogWrite(IA_Proximal, pwm);
      PORTB &= (~(1<<IB_Proximal));
    }else if(pwm < 0){//If PWM in negative 
      PORTB |= (1<<IB_Proximal);//IB is set high
      analogWrite(IA_Proximal, 255+pwm);//IA is a PWM of 255 minus the negative PWM
    }
  }else if(motorNum == 2){
    if (pwm >= 0){//If PWM is positive
      analogWrite(IA_Tension, pwm);
      PORTA &= (~(1<<IB_Tension));
    }else if(pwm < 0){//If PWM in negative 
      PORTA |= (1<<IB_Tension);//IB is set high
      analogWrite(IA_Tension, 255+pwm);//IA is a PWM of 255 minus the negative PWM
    }
  }
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

int MmPID(int error){//MCP joint PID
  //dt = millis() - previousTime;//time elapsed
  if(abs(error) < 2)error = 0;
  integral += error * dt;
  if(integral > integralUB){//Sets integral limits to prevent integral windup
    integral = integralUB;
  }else if(integral < integralLB){
    integral = integralLB;
  }else if(error < 4 && error > -4){
    integral *= 0.3;//0.2
  }
  
  derivative = (error - previous_error)/dt;
  double output = Kp*error + Ki*integral + Kd*derivative;
  previous_error = error;
 
  if(output > 255){
    output = 255;
  }else if(output < -255){
    output = -255;
  }else if(output > 2 && output < minPWMpos){
    output = minPWMpos;
  }else if(output < -2 && output > minPWMneg){
    output = minPWMneg;
  }
  
  int PWMout = round(output);
  return PWMout;//Returns the PID output
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

int MpPID(int error){//Proximal joint PID
  if(abs(error) < 2)error = 0;
  integral1 += error * dt1;
  if(integral1 > integralUB){//Sets integral limits to prevent integral windup
    integral1 = integralUB;
  }else if(integral1 < integralLB){
    integral1 = integralLB;
  }else if(error < 4 && error > -4){
    integral1 *= 0.3;
  }
  
  derivative1 = (error - previous_error1)/dt1;
  double output = Kp1*error + Ki1*integral1 + Kd1*derivative1;
  previous_error1 = error;
  
  if(output > 255){
    output = 255;
  }else if(output < -255){
    output = -255;
  }else if(output > 2 && output < minPWMpos1){
    output = minPWMpos1;
  }else if(output < -2 && output > minPWMneg1){
    output = minPWMneg1;
  }
  
  int PWMout = round(output);
  return PWMout;//Returns the PID output
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void setFingerPos(int MmTheta, int MpTheta){
  if(MmTheta < 0 || MmTheta > 90 ||  MpTheta < 0 ||  MpTheta > 90)return;
  updateJointValues();
  int Mm_error = MmTheta - ::MCP_angle;
  int Mp_error = MpTheta - ::proximal_angle;
  
  int Mm_PWM = MmPID(Mm_error);
  int Mp_PWM = MpPID(Mp_error);

  //limits the PWMs to match the Maximum the tension line can operate
  if(abs(Mm_PWM + Mp_PWM) > 255){
    Mm_PWM *= ((double)Mm_PWM / ((double)Mm_PWM + (double)Mp_PWM));
    Mp_PWM *= ((double)Mm_PWM / ((double)Mm_PWM + (double)Mp_PWM));
  }
  fingerMotorControl(Mm_PWM, Mp_PWM);
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void fingerMotorControl(int Mm_PWM, int Mp_PWM){
  int Mt_PWM = Mm_PWM + Mp_PWM;
  if(abs(Mt_PWM) > 255){
    return;//check the values are in range
  }
  if(Mt_PWM > 0){
    Mt_PWM *= 0.7;
  }else if(Mt_PWM < 0){
    Mm_PWM *= 0.5;
    Mp_PWM *= 0.6;
  }
  motorDriver(Mt_PWM, 2);//Tension line
  motorDriver(Mp_PWM, 1);//Proximal line
  motorDriver(Mm_PWM, 0);//MCP line
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void Co_Contraction(int percentageCC){
  if(percentageCC > 100) return;
  if(percentageCC < 10){
    motorDriver(0, 2);//Tension line
    motorDriver(0, 1);//Proximal line
    motorDriver(0, 0);//MCP line
    return;
  }
  
  int Mt_PWM = map(percentageCC, 10, 100, -20, -60);
  int Mp_PWM = map(percentageCC, 10, 100, 30, 255);
  int Mm_PWM = map(percentageCC, 0, 100, 0, 255);
  motorDriver(Mt_PWM, 2);//Tension line
  motorDriver(Mp_PWM, 1);//Proximal line
  motorDriver(Mm_PWM, 0);//MCP line
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void grip(int loosen, int Mm_percentage, int Mp_percentage){
  if(Mm_percentage < 0 || Mm_percentage > 100 || Mp_percentage < 0 || Mp_percentage > 100)return;
  
  int Mp_PWM = map(Mp_percentage, 0, 100, 0, 255);
  int Mm_PWM = map(Mm_percentage, 0, 100, 0, 255);
  
  if(abs(Mm_PWM + Mp_PWM) > 255){//Scales the max pwms
    Mm_PWM *= ((double)Mm_PWM / ((double)Mm_PWM + (double)Mp_PWM));
    Mp_PWM *= ((double)Mm_PWM / ((double)Mm_PWM + (double)Mp_PWM));
  }
  int Mt_PWM = (Mm_PWM + Mp_PWM) * 0.7;
  motorDriver(Mt_PWM, 2);//Tension line
  motorDriver(Mp_PWM, 1);//Proximal line
  motorDriver(Mm_PWM, 0);//MCP line
  delay(loosen);
  motorDriver(0, 2);//Turn the tension line off
  grip_flag = false;
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void co_contraction_PID(int co_contraction){
  if(!PID_cc_flag)return;
  unsigned long startTime = millis();//get current time
  while(millis() - startTime < 800  ){//loop for 800ms
    setFingerPos(MCP_setPoint, proximal_setPoint);//The PPAP control
  }
  Co_Contraction(co_contraction);//Sets the co_contraction
  PID_cc_flag = false;
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void tighten_pulleys(int ms){
  motorDriver(-70, 2);//Tension line
  motorDriver(70, 1);//Proximal line
  motorDriver(70, 0);//MCP line
  delay(ms);
  motorDriver(0, 2);//Tension line
  motorDriver(0, 1);//Proximal line
  motorDriver(0, 0);//MCP line
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup()
{
//Set up all pins to the correct mode
  pinMode(IA_MCP, OUTPUT);
  DDRB |= (1<<DDB0);//Sets the pins data direction register to write (output)
  pinMode(IA_Proximal, OUTPUT);
  DDRB |= (1<<DDB1);//Sets the pins data direction register to write (output)
  pinMode(IA_Tension, OUTPUT);
  DDRA |= (1<<DDA3);//Sets the pins data direction register to write (output)

  DDRA &= (~(1<<Proximal_pin));//Sets the pin to an input
  DDRA &= (~(1<<MCP_pin));//Sets the pin to an input

  DIDR0 |= (1<<ADC1D);//Disable digital register on the analog pin1
  DIDR0 |= (1<<ADC2D);//Disable digital register on the analog pin2
  
  //Ensure all pins are set low
  analogWrite(IA_MCP, 0);
  PORTB &= (~(1<<IB_MCP));//sets pin low
  analogWrite(IA_Proximal, 0);
  PORTB &= (~(1<<IB_Proximal));//sets pin low
  analogWrite(IA_Tension, 0);
  PORTA &= (~(1<<IB_Tension));//sets pin low
  
  tighten_pulleys(3300);
  
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);

}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop()
{  
  if(PID_flag){
    setFingerPos(MCP_setPoint, proximal_setPoint);
  }else if(PID_cc_flag){
    co_contraction_PID(co_contraction);
  }else if(grip_flag){
    grip(800, 70, 70);
  }
//Might need to add this into the pidcc loop
  TinyWireS_stop_check();//Needs to be a tight loop apparently
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
