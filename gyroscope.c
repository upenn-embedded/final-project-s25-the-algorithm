/* 
 * File:   gyroscope.c
 * Author: derek
 *
 * Created on April 22, 2025, 6:38 PM
 */
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#define F_CPU 16000000UL
#define SCL_CLOCK 100000L

void TWI_init(void) {
    // Set SCL frequency
    TWSR = 0x00;
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;

    // Enable TWI
    TWCR = (1<<TWEN);
}

void TWI_start(void) {
    TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void TWI_stop(void) {
    TWCR = (1<<TWSTO)|(1<<TWEN)|(1<<TWINT);
}

void TWI_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}


uint8_t TWI_read_ack(void) {
    TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint8_t TWI_read_nack(void) {
    TWCR = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
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
