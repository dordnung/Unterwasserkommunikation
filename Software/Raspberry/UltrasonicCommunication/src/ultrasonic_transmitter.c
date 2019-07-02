#include "ultrasonic_communication.h"
#include "ultrasonic_transmitter.h"

#include <stdlib.h>
#include <string.h>
#include <pigpio.h>

#define MAX_BYTES_PER_TRANSMIT 31

// Enum with possible I2C modes
enum i2c_mode {
    I2C_MODE_TRANSMIT = 0x01,
    I2C_MODE_TRANSMIT_BIT,
    I2C_MODE_SET_FREQUENCY_LOW,
    I2C_MODE_SET_FREQUENCY_HIGH,
    I2C_MODE_SET_FREQUENCY_BOTH,
    I2C_MODE_SET_DURATION_US,
    I2C_MODE_SET_DURATION_CYCLES,
    I2C_MODE_SET_WAIT_TIME
};

// Helper functions
static int ultrasonic_transmitter_write_2_bytes(uint8_t i2c_mode, uint16_t value);
static int ultrasonic_transmitter_write_4_bytes(uint8_t i2c_mode, uint16_t value1, uint16_t value2);
static int ultrasonic_transmitter_get_status();
static void ultrasonic_transmitter_wait_for_data_transmitted();

// Store transmission end pin, the i2c handle and whether we want to request status
static int set_transmission_end_pin = 0;
static int set_i2c_handle = -1;
static int set_request_status = 0;

int ultrasonic_transmitter_init(uint8_t transmission_end_pin, unsigned int slave_address, uint8_t request_status, uint8_t i2c_bus) {
    // Be sure the library itself is initialised
    if (!is_ultrasonic_communication_library_init()) {
        return PI_INIT_FAILED;
    }

    // Terminate if a transmission end pin or i2c handle is already set
    if (set_transmission_end_pin || set_i2c_handle >= 0) {
        ultrasonic_transmitter_terminate();
    }

    // Set the transmission end pin as an input pin
    int error;
    if ((error = gpioSetMode(transmission_end_pin, PI_INPUT)) < 0) {
        return error;
    }
    set_transmission_end_pin = transmission_end_pin;

    // Open a new I2C connection to the slave address
    if ((error = i2cOpen(i2c_bus, slave_address, 0)) < 0) {
        set_transmission_end_pin = 0;
        return error;
    }

    // The error is actually our i2c handle
    set_i2c_handle = error;
    set_request_status = request_status;

    return 0;
}

void ultrasonic_transmitter_terminate() {
    // Close the I2C handle if a connection is opened
    if (set_i2c_handle >= 0) {
        i2cClose(set_i2c_handle);
    }

    set_transmission_end_pin = 0;
    set_i2c_handle = -1;
    set_request_status = 0;
}

int ultrasonic_transmitter_set_frequency_low(uint16_t frequency) {
    return ultrasonic_transmitter_write_2_bytes(I2C_MODE_SET_FREQUENCY_LOW, frequency);
}

int ultrasonic_transmitter_set_frequency_high(uint16_t frequency) {
    return ultrasonic_transmitter_write_2_bytes(I2C_MODE_SET_FREQUENCY_HIGH, frequency);
}

int ultrasonic_transmitter_set_frequencies(uint16_t frequency_low, uint16_t frequency_high) {
    return ultrasonic_transmitter_write_4_bytes(I2C_MODE_SET_FREQUENCY_BOTH, frequency_low, frequency_high);
}

int ultrasonic_transmitter_set_duration_microseconds(uint16_t duration) {
    return ultrasonic_transmitter_write_2_bytes(I2C_MODE_SET_DURATION_US, duration);
}

int ultrasonic_transmitter_set_duration_cycles(uint16_t duration) {
    return ultrasonic_transmitter_write_2_bytes(I2C_MODE_SET_DURATION_CYCLES, duration);
}

int ultrasonic_transmitter_set_wait_time(uint16_t wait_time) {
    return ultrasonic_transmitter_write_2_bytes(I2C_MODE_SET_WAIT_TIME, wait_time);
}

int ultrasonic_transmit_bit(uint8_t bit) {
    // Check if I2C handle is opened
    if (set_i2c_handle < 0) {
        return PI_NO_HANDLE;
    }

    // Set mode and transmit the bit with I2C
    char buffer[2];
    buffer[0] = I2C_MODE_TRANSMIT_BIT;
    buffer[1] = bit;

    // Write to the I2C device
    int error;
    if ((error = i2cWriteDevice(set_i2c_handle, buffer, 2)) < 0) {
        return error;
    }

    // Wait until everything was transmitted
    ultrasonic_transmitter_wait_for_data_transmitted();

    return 0;
}

int ultrasonic_transmit_bytes(const uint8_t *bytes, uint8_t number_of_bytes, uint8_t wait_time_between_transmissions) {
    // Check if I2C handle is opened
    if (set_i2c_handle < 0) {
        return PI_NO_HANDLE;
    }

    // Calculate start and end
    uint8_t *current = (uint8_t *)bytes;
    uint8_t *end = current + number_of_bytes;

    // While not all bytes were transmitted
    while (current < end) {
        // Wait until next transmission if wanted (but only if not first)
        if (current != bytes && wait_time_between_transmissions > 0) {
            gpioSleep(PI_TIME_RELATIVE, 0, wait_time_between_transmissions * 1000);
        }

        // Calculate number of bytes to transmit on next transmission
        uint8_t num_bytes = (current + MAX_BYTES_PER_TRANSMIT > end) ? (end - current) : MAX_BYTES_PER_TRANSMIT;

        // Create buffer with bytes to transmit and one extra item for the transmit mode
        char *buffer = (char *)malloc((num_bytes + 1) * sizeof(char));
        memmove(buffer + 1, current, num_bytes);

        // Set the mode
        buffer[0] = I2C_MODE_TRANSMIT;

        // Write the bytes to the i2c device
        int error;
        if ((error = i2cWriteDevice(set_i2c_handle, buffer, num_bytes + 1)) < 0) {
            free(buffer);
            return error;
        }

        // Wait until everything was transmitted
        ultrasonic_transmitter_wait_for_data_transmitted();

        // Go on with next bytes
        free(buffer);
        current += num_bytes;
    }

    return 0;
}

int ultrasonic_transmit_string(const char *str, uint8_t wait_time_between_transmissions) {
    // Just use the transmit bytes method
    return ultrasonic_transmit_bytes((uint8_t *)str, strlen(str), wait_time_between_transmissions);
}

int ultrasonic_transmitter_write_2_bytes(uint8_t i2c_mode, uint16_t value) {
    // Check if I2C handle is opened
    if (set_i2c_handle < 0) {
        return PI_NO_HANDLE;
    }

    // Set mode and higher and lower byte
    char buffer[3];
    buffer[0] = i2c_mode;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = value & 0xFF;

    // Write to the I2C device
    int error;
    if ((error = i2cWriteDevice(set_i2c_handle, buffer, 3)) < 0) {
        return error;
    }

    // Return status if requested
    if (set_request_status) {
        return ultrasonic_transmitter_get_status();
    }

    return 0;
}

int ultrasonic_transmitter_write_4_bytes(uint8_t i2c_mode, uint16_t value1, uint16_t value2) {
    // Check if I2C handle is opened
    if (set_i2c_handle < 0) {
        return PI_NO_HANDLE;
    }

    // Set mode and higher and lower bytes
    char buffer[5];
    buffer[0] = i2c_mode;
    buffer[1] = (value1 >> 8) & 0xFF;
    buffer[2] = value1 & 0xFF;
    buffer[3] = (value2 >> 8) & 0xFF;
    buffer[4] = value2 & 0xFF;

    // Write to the I2C device
    int error;
    if ((error = i2cWriteDevice(set_i2c_handle, buffer, 5)) < 0) {
        return error;
    }

    // Return status if requested
    if (set_request_status) {
        return ultrasonic_transmitter_get_status();
    }

    return 0;
}

int ultrasonic_transmitter_get_status() {
    // Just read a byte
    return i2cReadByte(set_i2c_handle);
}

void ultrasonic_transmitter_wait_for_data_transmitted() {
    // Wait until transmission end pin is pulled up and down again
    while (gpioRead(set_transmission_end_pin));
    while (!gpioRead(set_transmission_end_pin));
}