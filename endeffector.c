/*
 * File:   main.c
 * Author: ericl
 *
 * Created on April 18, 2025, 10:59 AM
 */


 #include <xc.h>
 #include <util/delay.h>
 
 void Initialize() {
     DDRB |= (1 << PB0); // pin IN1 of HW-095 motor driver
     DDRB |= (1 << PB1); // pin IN2 of HW-095 motor driver
     
     // input pin (mimics commands in overall code)
     // if PB2 is high, motor spins in one direction, if PB2 is low, motor spins in opposite direction
     DDRB &= (1 << PB2);
     
     // PB3 is the kill switch. Motor will spin IF AND ONLY IF PB3 is high
     DDRB &= (1 << PB3); 
     
 }
 
 int main(void) {
     while (1) {
         if (PINB & (1 << PINB3)) {
             if (PINB & (1 << PINB2)) {
                 PORTB |= (1 << PB0);
                 PORTB &= ~(1 << PB1);
             } else {
                 PORTB &= ~(1 << PB0);
                 PORTB |= (1 << PB1);
             }
         } else {
             PORTB &= ~(1 << PB0);
             PORTB &= ~(1 << PB1);
         }
     }
 }
 