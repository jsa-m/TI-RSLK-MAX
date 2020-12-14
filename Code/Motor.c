// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.

#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
  // write this as part of Lab 13
    PWM_Init34(7500, 0, 0); //Set pins 2.6 and 2.7 as output, PWM's (10 ms period, low)
    P3->DIR |= 0xC0;    //Pins 3.6 (nSLPR) and 3.7 (nSLPL) as outputs
    P5->DIR |= 0x30;    //Pins 5.4 (DIRL) and 5.5 (DIRR) as outputs
    P5->OUT &= ~0x30;   //DIRL and DIRR to zero
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){
    P1->OUT &= ~0xC0;
    P3->OUT &= ~0xC0;   // low current sleep mode
    PWM_Duty3(0);       //Righ motor duty to 0
    PWM_Duty4(0);       //Left motor duty to 0
}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 7498)
//        rightDuty duty cycle of right wheel (0 to 7498)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){ 
    uint16_t duty;
    P5->OUT &= ~0x30;   //DIRL and DIRR to zero (PH left and right to zero)
    P3->OUT |= 0xC0;    //nSleep to 1
    if(leftDuty > 7498){
        duty = 7498;       //For 10ms max CCR[3] value is 14998
    }
    else{
        duty = leftDuty;
    }
    PWM_Duty3(duty);         //Sets duty of PWM motor right
    if(rightDuty > 7498){
        duty = 0;       //For 10ms max CCR[3] value is 14998
    }
    else{
        duty = rightDuty;
    }
    PWM_Duty4(duty);        //Sets duty of PWM motor left
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 7498)
//        rightDuty duty cycle of right wheel (0 to 7498)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){ 
    uint16_t duty;
    //MOTOR LEFT FORDWARD
    P5->OUT &= ~0x10;   //DIRL to zero (PH left to zero)
    P3->OUT |= 0x80;    //nSleep to 1 left motor
    if(leftDuty > 7498){
            duty = 0;       //For 10ms max CCR[3] value is 7498
        }
    else{
            duty = leftDuty;
            }
    PWM_Duty4(duty);        //Sets duty of PWM motor left
    //MOTOR RIGHT BACKWARD
    P5->OUT |= 0x20;   //DIRR to zero (PH right to one)
    P3->OUT |= 0x40;    //nSleep to 1
    if(rightDuty > 7498){
            duty = 0;       //For 10ms max CCR[3] value is 7498
        }
    else{
            duty = rightDuty;
            }
    PWM_Duty3(duty);    //Sets duty of PWM motor right

}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 7498)
//        rightDuty duty cycle of right wheel (0 to 7498)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){ 
    uint16_t duty;
    //MOTOR RIGHT Forward
    P5->OUT &= ~0x20;   //DIRR to zero (PH right to one)
    P3->OUT |= 0x40;    //nSleep to 1
    if(rightDuty > 7498){
            duty = 0;       //For 10ms max CCR[3] value is 7498
        }
    else{
            duty = rightDuty;
            }
    PWM_Duty3(duty);    //Sets duty of PWM motor right
    //MOTOR LEFT BACKWARD
    P5->OUT |= 0x10;   //DIRL to zero (PH left to zero)
    P3->OUT |= 0x80;    //nSleep to 1 left motor
    if(leftDuty > 7498){
            duty = 0;       //For 10ms max CCR[3] value is 14998
        }
    else{
            duty = leftDuty;
            }
    PWM_Duty4(duty);        //Sets duty of PWM motor left
}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 7498)
//        rightDuty duty cycle of right wheel (0 to 7498)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){ 
    uint16_t duty;
    P5->OUT |= 0x30;   //DIRL and DIRR to one (PH left and right to 1)
    P3->OUT |= 0xC0;    //nSleep to 1
    if(leftDuty > 7498){
          duty = 7498;       //For 10ms max CCR[3] value is 14998
        }
    else{
          duty = leftDuty;
        }
    PWM_Duty3(duty);         //Sets duty of PWM motor right
    if(rightDuty > 7498){
          duty = 7498;       //For 10ms max CCR[3] value is 14998
        }
    else{
          duty = rightDuty;
        }
    PWM_Duty4(duty);        //Sets duty of PWM motor left

}