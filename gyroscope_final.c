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

    while (1) {
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

        if (fabsf(avg_pitch) > 30.0f || fabsf(avg_roll) > 30.0f) {
            printf("Orientation exceeds ±30°!\n");
            PORTB ^= (1 << PB5); // Toggle LED
            PORTB ^= (1 << PB4); // Toggle LED
        }

        _delay_ms(100);
    }
}
