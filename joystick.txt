/* THIS IS BEN'S CODE PLUS CODE FOR JOYSTICK CONTROL FOR THE 3 STEPPER MOTORS */
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

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
    DDRD |= (1 << PD7);
}

void set_dir_ccw(){
    PORTB |= (1 << PB1);
    PORTB |= (1 << PB3);
    PORTD |= (1 << PD7);
    // PORTD &= ~(1 << PD1);
}

void set_dir_cw(){
    PORTB &= ~(1 << PB1);
    PORTB &= ~(1 << PB3);
    // PORTD |= (1 << PD1);
    PORTD &= ~(1 << PD7);
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

// Redirect printf to UART
int uart_putchar(char c, FILE *stream) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
    return 0;
}

void uart_init() {
    uint16_t ubrr = 103; // For 9600 baud with 16MHz
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;

    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    static FILE uart_stdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &uart_stdout;
}

void adc_init() {
    PRR0 &= ~(1 << PRADC);
    ADMUX = (1 << REFS0); // AVcc
    ADCSRA = (1 << ADEN)  // Enable ADC
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
    DIDR0 = (1 << ADC0D); // Disable digital input
}

uint16_t adc_read() {
    ADCSRA |= (1 << ADSC);              // Start
    while (ADCSRA & (1 << ADSC));       // Wait
    return ADCW;
}

int main(int argc, char** argv) {
   
    Initialize();
    
    uart_init();
    adc_init();

    while (1) {
        uint16_t adcVal = adc_read();
        printf("ADCVAL = %d\r\n", adcVal);

        if (adcVal > 300) {
            set_dir_cw();
            send_pulse();
        } else if (adcVal < 100) {
            set_dir_ccw();
            send_pulse();
        }
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB2);
        PORTD &= ~(1 << PD0);
    }
}