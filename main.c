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
   
    DDRD |= (1 << PD0);
    DDRD |= (1 << PB1);
}

void set_dir_ccw(){
    PORTB |= (1 << PB1);
    PORTB |= (1 << PB3);
    PORTD |= (1 << PD1);
}

void set_dir_cw(){
    PORTB &= ~(1 << PB1);
    PORTB &= ~(1 << PB3);
    PORTD &= ~(1 << PD1);
}

void send_pulse() {
    // Send a pulse
    PORTB |= (1 << PB0);
    PORTB |= (1 << PB2);
    PORTD |= (1 << PD0);
    _delay_us(10000);
    PORTB &= ~(1 << PB0);
    PORTB &= ~(1 << PB2);
    PORTD &= ~(1 << PD0);
    _delay_us(10000);
}

/*
 *
 */
int main(int argc, char** argv) {
   
    Initialize();
    
    _delay_us(10000000);
    
    PORTB |= (1 << PB1);
    PORTB |= (1 << PB3);
    PORTD |= (1 << PD1);
   
    while(1) {
        set_dir_cw();
        for (int i = 0; i < 1000; i++) {
            send_pulse();
        }
        set_dir_ccw();
        for (int i = 0; i < 1000; i++) {
            send_pulse();
        }
    }
    
//    set_dir_ccw();
//    for (int i = 0; i < 500; i++) {
//        send_pulse();
//      }
}
