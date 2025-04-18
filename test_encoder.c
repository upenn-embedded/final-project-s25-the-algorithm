/*
 * File:   main.c
 * Author: ericl
 *
 * Created on April 18, 2025, 10:59 AM
 */


 #include <xc.h>
 #include <util/delay.h>
 #include <avr/interrupt.h>

#define CPR 60
#define DURATION 100
#define Kp 1.5

long prev_encoder_count = 0;
float desired_rpm = 50.0;
volatile long encoder_count = 0;

void control_loop() {
    long current_encoder = encoder_count;
    long delta = current_encoder - prev_encoder_count;
    prev_encoder_count = current_encoder;

    // Calculate actual RPM
    float actual_rpm = ((float)delta / CPR) * (60000.0 / DURATION);

    // Compute error
    float error = desired_rpm - actual_rpm;

    // Adjust PWM
    float pwm_output = OCR0A + Kp * error;

    // Set boundary conditions to [0, 255]
    if (pwm_output > 255) pwm_output = 255;
    if (pwm_output < 0) pwm_output = 0;

    OCR0A = (uint8_t) pwm_output;
}

void pwm_init() {
    DDRD |= (1 << PD6); // OC0A as output
    TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // Fast PWM, non-inverting
    TCCR0B |= (1 << CS01) | (1 << CS00); // Prescaler = 64
    OCR0A = 128; // Start with ~50% duty
}


void encoder_init() {
    DDRD &= ~((1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD2) | (1 << PD3); // Pull-ups (optional)

    EICRA |= (1 << ISC00); // Detect any rising / falling edge on INT0
    EIMSK |= (1 << INT0);
    sei();
}

ISR(INT0_vect) {
    if (PIND & (1 << PD3))
        encoder_count++;
    else
        encoder_count--;
}

 
 int main(void) {
    pwm_init();
    encoder_init();
    
    // this will mess with PWM / duty cycle 
    OCR0A = 0;
    _delay_ms(1000);
    OCR0A = 100;
    _delay_ms(1000);
    OCR0A = 200;
    _delay_ms(1000);
    OCR0A = 255;


    DDRB |= (1 << PB0); // IN1
    DDRB |= (1 << PB1); // IN2

    // Set forward direction
    PORTB |= (1 << PB0);
    PORTB &= ~(1 << PB1);

    while (1) {
        control_loop();
        _delay_ms(DURATION);
    }
}

 
 
