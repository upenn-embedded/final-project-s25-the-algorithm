/* 
 * File:   main.c
 * Author: derek
 *
 * Created on April 12, 2025, 4:57 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/delay.h>


void Initialize() {
    DDRB |= (1 << PB0);
    DDRB |= (1 << PB1);
    
    DDRB |= (1 << PB2);
    DDRB |= (1 << PB3);
    
    DDRB |= (1 << PB4);
    DDRB |= (1 << PB5);
}

/*
 * 
 */
int main(int argc, char** argv) {
    
    Initialize();

    while (1) {
        // Set directions
        PORTB |= (1 << PB1);
        PORTB |= (1 << PB3);
        PORTB |= (1 << PB5);
        // Send a pulse
        PORTB |= (1 << PB0);
        PORTB |= (1 << PB2);
        PORTB |= (1 << PB4);
        _delay_us(175);
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB2);
        PORTB &= ~(1 << PB4);
        _delay_us(175);
    }
    return (EXIT_SUCCESS);
}

