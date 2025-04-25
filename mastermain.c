#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>


// #define STEPPER_MOTOR_CONTROL
#define STEPPER_MOTOR_CONTROL_AND_JOYSTICK
// #define END_EFFECTOR_INTERRUPT
// #define END_EFFECTOR_POLLING
// #define GYRO

#ifdef STEPPER_MOTOR_CONTROL
/* THIS IS BEN'S ORIGINAL UNMODIFIED CODE, WHICH HAD THE END EFFECTOR OSCILLATE UP AND DOWN */
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
}
#endif



#ifdef STEPPER_MOTOR_CONTROL_AND_JOYSTICK
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
    
    _delay_us(10000000);
    
    PORTB |= (1 << PB1);
    PORTB |= (1 << PB3);
    PORTD |= (1 << PD1);
    
    uart_init();
    adc_init();

    DDRB |= (1 << PB0) | (1 << PB1);

    while (1) {
        uint16_t adcVal = adc_read();
        printf("ADCVAL = %d\r\n", adcVal);

        if (adcVal > 800) {
            set_dir_cw();
            send_pulse();
        } else if (adcVal < 300) {
            set_dir_ccw();
            send_pulse();
        }
    }
}
#endif

/* THIS IS THE CODE FOR THE END EFFECTOR MOTOR */


#ifdef END_EFFECTOR_POLLING
#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

void Initialize() {
    DDRB |= (1 << PB0) | (1 << PB1);  // IN1 and IN2 as output
    DDRB &= ~((1 << PB2) | (1 << PB3)); // PB2 and PB3 as input

    PORTB |= (1 << PB2) | (1 << PB3); // Enable pull-ups on inputs (optional)

    PCICR |= (1 << PCIE0);    // Enable pin change interrupt for PCINT[7:0]
    PCMSK0 |= (1 << PCINT2);  // Enable interrupt on PB2 (PCINT2)
    
    // Setup for ADC (10bit = 0-1023)
    // Clear power reduction bit for ADC
    PRR0 &= ~(1 << PRADC);

    // Select Vref = AVcc
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);

    // Set the ADC clock div by 128
    // 16M/128=125kHz
    ADCSRA |= (1 << ADPS0);
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS2);

    // Select Channel ADC0 (pin C0)
    ADMUX &= ~(1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX3);

    ADCSRA |= (1 << ADATE); // Autotriggering of ADC

    // Free running mode ADTS[2:0] = 000
    ADCSRB &= ~(1 << ADTS0);
    ADCSRB &= ~(1 << ADTS1);
    ADCSRB &= ~(1 << ADTS2);

    // Disable digital input buffer on ADC pin
    DIDR0 |= (1 << ADC0D);

    // Enable ADC
    ADCSRA |= (1 << ADEN);

    // Start conversion
    ADCSRA |= (1 << ADSC);
}

int main(void) {
    Initialize();
    PORTB &= ~(1 << PB0);
    PORTB &= ~(1 << PB1);
    while (1) {
        uint16_t adcVal = ADC;
        printf("ADCVAL = %d\n\n", adcVal);
        if (adcVal > 800) {
            PORTB &= ~(1 << PB0);
            PORTB |= (1 << PB1);
        } else if (adcVal < 300) {
            PORTB |= (1 << PB0);
            PORTB &= ~(1 << PB1);
        } else {
            PORTB &= ~(1 << PB0);
            PORTB &= ~(1 << PB1);
        }
    }

#endif
    

#ifdef END_EFFECTOR_INTERRUPT
/* THIS IS THE CODE FOR THE END EFFECTOR MOTOR IN INTERRUPT MODE */
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
#endif



#ifdef GYRO
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#define F_CPU 16000000UL
#define SCL_CLOCK 100000L

void TWI_init(void) {
    // Set SCL frequency
    TWSR0 = 0x00;
    TWBR0 = ((F_CPU/SCL_CLOCK)-16)/2;

    // Enable TWI
    TWCR0 = (1<<TWEN);
}

void TWI_start(void) {
    TWCR0 = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
    while (!(TWCR0 & (1<<TWINT)));
}

void TWI_stop(void) {
    TWCR0 = (1<<TWSTO)|(1<<TWEN)|(1<<TWINT);
}

void TWI_write(uint8_t data) {
    TWDR0 = data;
    TWCR0 = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR0 & (1<<TWINT)));
}


uint8_t TWI_read_ack(void) {
    TWCR0 = (1<<TWEN)|(1<<TWINT)|(1<<TWEA);
    while (!(TWCR0 & (1<<TWINT)));
    return TWDR0;
}

uint8_t TWI_read_nack(void) {
    TWCR0 = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR0 & (1<<TWINT)));
    return TWDR0;
}


uint8_t gyro_read_register(uint8_t reg) {
    uint8_t data;
    TWI_start();
    TWI_write(0x68 << 1);
    TWI_write(reg);
    TWI_start();
    TWI_write((0x68 << 1) | 1);
    data = TWI_read_nack();
    TWI_stop();
    return data;
}

void main() {
    gyro_read_register(0);
}

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
#endif