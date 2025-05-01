/*
 * File:   end_effector.c
 * Author: ericl
 *
 * Created on April 18, 2025, 10:59 AM
 */

// This code controls the end effector motor through the HW-095 motor driver

#include <xc.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void Initialize() {
    DDRB |= (1 << PB0) | (1 << PB1);  // IN1 and IN2 as output
    DDRB &= ~((1 << PB2) | (1 << PB3)); // PB2 and PB3 as input

    PORTB |= (1 << PB2) | (1 << PB3); // Enable pull-ups on inputs (optional)

    PCICR |= (1 << PCIE0);    // Enable pin change interrupt for PCINT[7:0]
    PCMSK0 |= (1 << PCINT2);  // Enable interrupt on PB2 (PCINT2)
    
    sei(); // Enable global interrupts
}

ISR(PCINT0_vect) {
    if (!(PINB & (1 << PB3))) { 
        // turn off end effector motor
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB1);
        return;
    }

    if (PINB & (1 << PB2)) {
        // Clockwise
        PORTB &= ~(1 << PB0);
        PORTB |= (1 << PB1);
    } else {
        // Counter clockwise
        PORTB |= (1 << PB0);
        PORTB &= ~(1 << PB1);
    }
}

int main(void) {
    Initialize();
    while (1);
}
