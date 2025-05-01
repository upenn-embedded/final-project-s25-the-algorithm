//#include <stdio.h>
//#include <stdlib.h>
//#include <avr/io.h>
//#include <avr/delay.h>
//#include <avr/interrupt.h>

#define STEPPER_MOTOR_CONTROL_AND_JOYSTICK
// #define END_EFFECTOR_INTERRUPT
// #define END_EFFECTOR_POLLING
// #define GYRO

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>
#include "uart.h"

#define F_CPU 16000000UL
#define SCL_CLOCK 100000L
#define RAD_TO_DEG 57.2957795f
#define FILTER_WINDOW 50

static void TWI_init(void);
static uint8_t TWI_start(void);
static void TWI_stop(void);
static uint8_t TWI_write(uint8_t data);
static uint8_t TWI_read_ack(void);
static uint8_t TWI_read_nack(void);

static uint8_t LSM_addr = 0;
static uint8_t gyro_read_register(uint8_t reg);
static void gyro_write_register(uint8_t reg, uint8_t data);
static int16_t accel_read_axis(uint8_t msb_reg);
static uint8_t gyro_detect_address(void);
static void gyro_init(void);

uint16_t adc_read();
void adc_init();
void uart_init_adc();
int uart_putchar(char c, FILE *stream);
void send_pulse();
void set_dir_cw();
void set_dir_ccw();
void Initialize();

// Buffers for filtered pitch and roll
float pitch_buffer[FILTER_WINDOW] = {0};
float roll_buffer[FILTER_WINDOW] = {0};
uint8_t filter_index = 0;

float moving_average(float *buffer) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < FILTER_WINDOW; i++) {
        sum += buffer[i];
    }
    return sum / FILTER_WINDOW;
}

static void TWI_init(void) {
    TWSR0 = 0x00;
    TWBR0 = ((F_CPU / SCL_CLOCK) - 16) / 2;
    TWCR0 = (1 << TWEN);
}

static uint8_t TWI_start(void) {
    TWCR0 = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR0 & (1 << TWINT)));
    return (TWSR0 & 0xF8);
}

static void TWI_stop(void) {
    TWCR0 = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

static uint8_t TWI_write(uint8_t data) {
    TWDR0 = data;
    TWCR0 = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR0 & (1 << TWINT)));
    return (TWSR0 & 0xF8);
}

static uint8_t TWI_read_ack(void) {
    TWCR0 = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
    while (!(TWCR0 & (1 << TWINT)));
    return TWDR0;
}

static uint8_t TWI_read_nack(void) {
    TWCR0 = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR0 & (1 << TWINT)));
    return TWDR0;
}

static uint8_t gyro_read_register(uint8_t reg) {
    TWI_start();
    TWI_write(LSM_addr << 1);
    TWI_write(reg);
    TWI_start();
    TWI_write((LSM_addr << 1) | 1);
    uint8_t val = TWI_read_nack();
    TWI_stop();
    return val;
}

void gyro_write_register(uint8_t reg, uint8_t data) {
    TWI_start();
    TWI_write(LSM_addr << 1);
    TWI_write(reg);
    TWI_write(data);
    TWI_stop();
    _delay_ms(2);
}

static int16_t accel_read_axis(uint8_t msb_reg) {
    uint8_t low = gyro_read_register(msb_reg - 1);
    uint8_t high = gyro_read_register(msb_reg);
    return (int16_t)((high << 8) | low);
}

static uint8_t gyro_detect_address(void) {
    uint8_t addr_list[2] = {0x6A, 0x6B};
    for (uint8_t i = 0; i < 2; i++) {
        LSM_addr = addr_list[i];
        if (gyro_read_register(0x0F) == 0x69) return 1;
    }
    return 0;
}

void gyro_init(void) {
    gyro_write_register(0x12, 0x44); // CTRL3_C = BDU=1, IF_INC=1
    _delay_ms(10);
    gyro_write_register(0x10, 0x4A); // CTRL1_XL = 104 Hz, ±4g
    _delay_ms(10);
}

void compute_acc_angles(float ax, float ay, float az, float *pitch, float *roll) {
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) norm = 1e-6f;

    ax /= norm;
    ay /= norm;
    az /= norm;

    *pitch = atan2f(-az, sqrtf(ax * ax + ay * ay)) * RAD_TO_DEG;
    *roll = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

int main(void) {
    uart_init();
    uart_init_adc();
    
    TWI_init();
    _delay_ms(100);
    

    if (!gyro_detect_address()) {
        printf("LSM6DS3 not found!\n");
        while (1);
    }
    

    gyro_init();
    DDRB |= (1 << PB5);
    DDRB |= (1 << PB4);
    
    
    // Calibration
    _delay_ms(2000);
    float pitch_offset = 0.0f, roll_offset = 0.0f;
    

    {
        int16_t ax_raw = accel_read_axis(0x29); // X
        int16_t ay_raw = accel_read_axis(0x2B); // Y
        int16_t az_raw = accel_read_axis(0x2D); // Z
        

        float ax = (float)ax_raw;
        float ay = (float)ay_raw;
        float az = (float)az_raw;

        compute_acc_angles(ax, ay, az, &pitch_offset, &roll_offset);
        printf("Calibration complete. Offset Pitch: %.2f deg, Offset Roll: %.2f deg\n\n", pitch_offset, roll_offset);
    }

    Initialize();
    
    adc_init();

    
    while (1) {
        
        uint16_t adcVal = adc_read();
        printf("ADCVAL = %d\r\n", adcVal);

        if (adcVal > 800) {
            set_dir_cw();
            send_pulse();
        } else if (adcVal < 200) {
            set_dir_ccw();
            send_pulse();
        } else {
            PORTB &= ~(1 << PB0);
            PORTB &= ~(1 << PB2);
            PORTD &= ~(1 << PD0);
        }
        
        int16_t ax_raw = accel_read_axis(0x29); // X
        int16_t ay_raw = accel_read_axis(0x2B); // Y
        int16_t az_raw = accel_read_axis(0x2D); // Z

        float ax = (float)ax_raw;
        float ay = (float)ay_raw;
        float az = (float)az_raw;

        float pitch = 0.0f, roll = 0.0f;
        compute_acc_angles(ax, ay, az, &pitch, &roll);

        pitch -= pitch_offset;
        roll  -= roll_offset;

        // Update buffers
        pitch_buffer[filter_index] = pitch;
        roll_buffer[filter_index] = roll;
        filter_index = (filter_index + 1) % FILTER_WINDOW;

        // Compute moving average
        float avg_pitch = moving_average(pitch_buffer);
        float avg_roll  = moving_average(roll_buffer);

        printf("Pitch: %.2f deg\tRoll: %.2f deg\n", avg_pitch, avg_roll);

        if (fabsf(avg_pitch) > 10.0f || fabsf(avg_roll) > 10.0f) {
            printf("Orientation exceeds ±10°!\n");
            PORTB ^= (1 << PB5); // Toggle LED
            PORTB ^= (1 << PB4); // Toggle LED
        }

         _delay_ms(100);
    }
}



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

void uart_init_adc() {
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

//int main(int argc, char** argv) {
//   
//    Initialize();
//    
//    uart_init();
//    adc_init();
//
//    while (1) {
//        uint16_t adcVal = adc_read();
//        printf("ADCVAL = %d\r\n", adcVal);
//
//        if (adcVal > 300) {
//            set_dir_cw();
//            send_pulse();
//        } else if (adcVal < 100) {
//            set_dir_ccw();
//            send_pulse();
//        }
//        PORTB &= ~(1 << PB0);
//        PORTB &= ~(1 << PB2);
//        PORTD &= ~(1 << PD0);
//    }
//}
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