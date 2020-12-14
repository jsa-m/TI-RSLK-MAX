/*
Low-level functions that interface bump switches on the robot.
*/

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"

void (*Port4Task)(uint8_t);   // Handle bumper interrupt

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt_Init(void(*task)(uint8_t)){
    Port4Task = task;   //Pointer to function
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;  //Pins 0,2,3,5,6,7 as GPIO
    P4->DIR &= ~0xED;   //Pins 0,2,3,5,6,7 as inputs
    P4->REN |= 0xED;
    P4->OUT |= 0xED;    //Pull up resistor for pins 0,2,3,5,6,7
    P4->IES |= 0xED;    //High to low edge interrupt select
    P4->IFG &= ~0xED;   //Clear interrupt flag
    P4->IE |= 0xED;    //Interrupt enable
    NVIC->IP[9] = ( NVIC->IP[9] & 0xFF1FFFFF) | 0x00400000; //Priority 2 PORT4 IRQ
    NVIC->ISER[1] |= 0x00000040;    //Enable interrupts NVIC port 4 which is interrupt 38 -> bit 6
}
// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    uint8_t data3,data2,data1,data;
    data3 = (P4->IN & 0xE0) >> 2;   //We have bit 7,6,5 read that are bit 5,4,3
    data2 = (P4->IN & 0X0C) >> 1;   //We have bit 3,2 read that are bit 2,1
    data1 = (P4->IN & 0x01);
    data = data3|data2|data1;
    data = ~data & ~0xC0;          //Positive logic result, bits 6,7 to zero (there are not part of the bumper)
    return data; // replace this line
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    P4->IFG &= ~0xED;       //Clear flag of port4 bumper pins;
    Port4Task(Bump_Read());
}