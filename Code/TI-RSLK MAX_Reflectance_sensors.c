/* 
Algorithms that provide functions to take measurements using the
QTRX reflectance sensor array

Author: Joaquin Sopena
Date: Nov, 2020
*/

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "..\inc\Clock.h"

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
//ONLY ONE SENSOR, FOR TESTING
/*
void Reflectance_Init(void){
    // write this as part of Lab 6
    P5->SEL0 = P5->SEL0 & ~0x08;
    P5->SEL1 = P5->SEL1 & ~0x08;        //Set pin 5.3 as GPIO
    P5->DIR = P5->DIR | 0x08;           //Set pin 5.3 as output
    P5->OUT = P5->OUT & ~0x08;          //Set pin 5.3 to low
    P7->DIR = P7->DIR & ~0x01;          //Set pin 7.0 as input
    P4->SEL0 = P4->SEL0 & ~0x01;
    P4->SEL1 = P4->SEL1 & ~0x01;        //Set pin 4.0 as GPIO
    P4->DIR = P4->DIR | 0x01;           //Set pin 4.0 as output
}
*/
//To test maybe the sensors need to be a little bit higher
void Reflectance_Init(void){
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;         //Set pin 5.3 as GPIO
    P5->DIR |= 0x08;           //Set pin 5.3 as output
    P5->OUT &= ~0x08;          //Set pin 5.3 to low
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;         //Set pin 9.2 as GPIO
    P9->DIR |= 0x04;           //Set pin 9.2 as output
    P9->OUT &= ~0x04;          //Set pin 9.2 to low
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;         //Set pin 7.0 to 7.7 as GPIO
    P7->DIR &= ~0xFF;          //Set pin 7.0 to 7.7 as input
}

// ------------Reflectance_Test------------
// Read one sensor
// Turn on the first IR LEDs
// Pulse the first sensor high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the first IR LEDs
// Output: None
// Assumes: Reflectance_Init() has been called
void Reflectance_Test(void){
int i = 0;
uint8_t data;
    // write this as part of Lab 6
  P5->OUT = P5->OUT | 0x08;             //Set P5.3 high (Turn on IR LED)
  P7->DIR = P7->DIR | 0x01;             //Set P7.0 an output
  P7->OUT = P7->OUT | 0x01;             //Set P7.0 high (charging the capacitor)
  Clock_Delay1us(10);                   //Wait 10 us
  P7->DIR = P7->DIR & ~0x01;            //Set pin 7.0 as input
  for(i=0; i<10000; i++)
  {
      data = P7->IN & 0x01;             //Read pin 7.0 (zero or one)
      P4->OUT = (P4->OUT & ~0x01) | data; //Output binary to P4.0 (see binary in real time)
  }
  P5->OUT = P5->OUT & ~0x08;            //Set P5.3 to low
}


// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time){
uint8_t result;
  P5->OUT |= 0x08;      //Set pin 5.3 to high
  P9->OUT |= 0x04;      //Set pin 9.2 to high (turn on IR led)
  P7->DIR = 0xFF;       //Set pins from 7.0 to 7.7 to output
  P7->OUT = 0xFF;       //Set pins from 7.0 to 7.7 to high to charge the capacitors
  Clock_Delay1us(10);   //Wait 10 us
  P7->DIR = 0x00;       //Change pins from 7.0 to 7.7 to input
  Clock_Delay1us(time); //if we wait 1000 μs, then the white will have decayed to a zero, while the black will still be high. This allows us to differentiate between white and black
  result = P7->IN;      //converts voltages into binary (0 white detected, 1 black detected)
  P5->OUT &=~0x08;      //Set P5.3 to low (save energy)
  P9->OUT &=~0x04;      //Set P9.2 to low(save energy)
  return result;
}


// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){
uint8_t result;
  P5->OUT |= 0x08;      //Set pin 5.3 to high
  P9->OUT |= 0x04;      //Set pin 9.2 to high (turn on IR led)
  P7->DIR |= 0x18;       //Set pins 7.3 and 7.4 to output (center sensors)
  P7->OUT |= 0x18;       //Set pins 7.3 and 7.4 to high to charge the capacitors
  Clock_Delay1us(10);   //Wait 10 us
  P7->DIR &= ~0x18;       //Change pins 7.3 and 7.4 to input
  Clock_Delay1us(time);   //if we wait 1000 μs, then the white will have decayed to a zero, while the black will still be high. This allows us to differentiate between white and black
  result = P7->IN & 0x18; //converts voltages into binary (0 white detected, 1 black detected)
  P5->OUT &=~0x08;      //Set P5.3 to low (save energy)
  P9->OUT &=~0x04;      //Set P9.2 to low(save energy)
  if(result == 0x18){
      return 3; //On road
  }
  else if(result == 0x08){
      return 1; //off to left
  }
  else if(result == 0x10){
      return 2; //off to right
  }
  else{
      return 0; //off road
  }

}


// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data){
int W[] = {334, 238, 142, 48, -48, -142, -238, -334}; //Positions defined as relative distance to the center of the robot in .1 mm
int b[] = {0, 0, 0, 0, 0, 0, 0, 0};                   //Sensor readings
int result = 0, numSenactive = 0;
int i;
  for(i=1;i<=8;i++){                                  //Vector to know which sensor is active
      result = (data & (1<<i-1));
      if(result != 0){
          b[i-1] = 1;
          numSenactive +=1;
      }
      else{
          b[i-1] = 0;
      }
  }
  result = 0;
  if(numSenactive > 0){                              //Calculate the weighted average distance
      for(i=0; i<8 ; i++){
          result = result + (b[i]*W[i]);
      }
      result = result / numSenactive;
  }
return result;                                       //Distance from the center of the robot
}

