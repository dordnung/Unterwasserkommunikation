#include "ultrasonic_communication.h"
#include "ultrasonic_receiver.h"

#include <stdlib.h>
#include <string.h>
#include <pigpio.h>

#define RECEIVER_ERROR         -200
#define MAX_BYTES_PER_TRANSMIT 31

// Enum with possible I2C modes
enum i2c_mode {
    I2C_MODE_RECEIVE = 0x01,
    I2C_MODE_RECEIVE_BIT,
    I2C_MODE_SET_FREQUENCY_LOW,
    I2C_MODE_SET_FREQUENCY_HIGH,
    I2C_MODE_SET_FREQUENCY_BOTH,
    I2C_MODE_SET_DURATION_US,
    I2C_MODE_SET_DURATION_CYCLES,
    I2C_MODE_SET_WAIT_TIME
};

// Helper functions
static int ultrasonic_receiver_write_2_bytes(uint8_t i2c_mode, uint16_t value);
static int ultrasonic_receiver_write_4_bytes(uint8_t i2c_mode, uint16_t value1, uint16_t value2);
static int ultrasonic_receiver_get_bytes(uint8_t *buffer, uint8_t num_bytes, uint8_t *num_bytes_received);
static int ultrasonic_receiver_get_status();
static void ultrasonic_receiver_wait_for_data_received();

// Store data received pin, the i2c handle and whether we want to request status
static int set_data_received_pin = 0;
static int set_i2c_handle = -1;
static int set_request_status = 0;

int ultrasonic_receiver_init(uint8_t data_received_pin, unsigned int slave_address, uint8_t request_status, uint8_t i2c_bus) {
    // Be sure the library itself is initialised
    if (!is_ultrasonic_communication_library_init()) {
        return PI_INIT_FAILED;
    }

    // Terminate if a data received pin or i2c handle is already set
    if (set_data_received_pin || set_i2c_handle >= 0) {
        ultrasonic_receiver_terminate();
    }

    // Set the data received pin as an input pin
    int error;
    if ((error = gpioSetMode(data_received_pin, PI_INPUT)) < 0) {
        return error;
    }
    set_data_received_pin = data_received_pin;

    // Open a new I2C connection to the slave address
    if ((error = i2cOpen(i2c_bus, slave_address, 0)) < 0) {
        set_data_received_pin = 0;
        return error;
    }

    // The error is actually our i2c handle
    set_i2c_handle = error;
    set_request_status = request_status;

    return 0;
}

void ultrasonic_receiver_terminate() {
    // Close the I2C handle if a connection is opened
    if (set_i2c_handle >= 0) {
        i2cClose(set_i2c_handle);
    }

    set_data_received_pin = 0;
    set_i2c_handle = -1;
    set_request_status = 0;
}

int ultrasonic_receiver_set_frequency_low(uint16_t frequency) {
    return ultrasonic_receiver_write_2_bytes(I2C_MODE_SET_FREQUENCY_LOW, frequency);
}

int ultrasonic_receiver_set_frequency_high(uint16_t frequency) {
    return ultrasonic_receiver_write_2_bytes(I2C_MODE_SET_FREQUENCY_HIGH, frequency);
}

int ultrasonic_receiver_set_frequencies(uint16_t frequency_low, uint16_t frequency_high) {
    return ultrasonic_receiver_write_4_bytes(I2C_MODE_SET_FREQUENCY_BOTH, frequency_low, frequency_high);
}

int ultrasonic_receiver_set_duration_microseconds(uint16_t duration) {
    return ultrasonic_receiver_write_2_bytes(I2C_MODE_SET_DURATION_US, duration);
}

int ultrasonic_receiver_set_duration_cycles(uint16_t duration) {
    return ultrasonic_receiver_write_2_bytes(I2C_MODE_SET_DURATION_CYCLES, duration);
}

int ultrasonic_receiver_set_wait_time(uint16_t wait_time) {
    return ultrasonic_receiver_write_2_bytes(I2C_MODE_SET_WAIT_TIME, wait_time);
}

int ultrasonic_receive_bit(uint8_t *bit, uint16_t timeout) {
    // Check if I2C handle is opened
    if (set_i2c_handle < 0) {
        return PI_NO_HANDLE;
    }

    // Set mode and timeout
    char buffer[3];
    buffer[0] = I2C_MODE_RECEIVE_BIT;
    buffer[1] = (timeout >> 8) & 0xFF;
    buffer[2] = timeout & 0xFF;

    // Write to the I2C device
    int error;
    if ((error = i2cWriteDevice(set_i2c_handle, buffer, 3)) < 0) {
        return error;
    }

    // Wait until bit was received
    ultrasonic_receiver_wait_for_data_received();

    // Get the received bit
    uint8_t bytes_received = 0;
    if ((error = ultrasonic_receiver_get_bytes(bit, 1, &bytes_received)) < 0) {
        return error;
    }

    // Check if we really got one byte/bit
    if (bytes_received != 1) {
        return RECEIVER_ERROR;
    }

    return 0;
}

int ultrasonic_receive_bytes(uint8_t *bytes, uint8_t number_of_bytes, uint16_t timeout, uint8_t *bytes_received) {
    // Check if I2C handle is opened
    if (set_i2c_handle < 0) {
        return PI_NO_HANDLE;
    }

    // Calculate start and end
    uint8_t *current = bytes;
    uint8_t *end = current + number_of_bytes;
    *bytes_received = 0;

    // Set mode and timeout
    char buffer[4];
    buffer[0] = I2C_MODE_RECEIVE;
    buffer[2] = (timeout >> 8) & 0xFF;
    buffer[3] = timeout & 0xFF;

    // While not all bytes were transmitted
    while (current < end) {
        // Calculate number of bytes to receive on next transmission
        uint8_t num_bytes = (current + MAX_BYTES_PER_TRANSMIT > end) ? (end - current) : MAX_BYTES_PER_TRANSMIT;
        buffer[1] = num_bytes;

        // Write the bytes to the i2c device
        int error;
        if ((error = i2cWriteDevice(set_i2c_handle, buffer, 4)) < 0) {
            return error;
        }

        // Wait until bytes were received
        ultrasonic_receiver_wait_for_data_received();

        // Get the received bytes
        uint8_t actual_bytes_received;
        if ((error = ultrasonic_receiver_get_bytes(current, num_bytes, &actual_bytes_received)) < 0) {
            return error;
        }

        // Check if all bytes were received
        *bytes_received += actual_bytes_received;
        if (actual_bytes_received != num_bytes) {
            return 0;
        }

        // Go on with next bytes
        current += num_bytes;
    }

    return 0;
}

int ultrasonic_receive_string(char *str, uint8_t str_length, uint16_t timeout, uint8_t *bytes_received) {
    // Just use the normal receive bytes method
    int error = ultrasonic_receive_bytes((uint8_t *)str, str_length, timeout, bytes_received);

    // Null terminate string
    str[*bytes_received] = '\0';
    return error;
}

int ultrasonic_receiver_write_2_bytes(uint8_t i2c_mode, uint16_t value) {
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
        return ultrasonic_receiver_get_status();
    }

    return 0;
}

int ultrasonic_receiver_write_4_bytes(uint8_t i2c_mode, uint16_t value1, uint16_t value2) {
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
        return ultrasonic_receiver_get_status();
    }

    return 0;
}

int ultrasonic_receiver_get_bytes(uint8_t *buffer, uint8_t num_bytes, uint8_t *num_bytes_received) {
    // Use a seperate receive buffer, as we got an extra byte
    char *receive_buffer = (char *)malloc((num_bytes + 1) * sizeof(char));

    // Read the number of wanted bytes
    int error = i2cReadDevice(set_i2c_handle, receive_buffer, num_bytes + 1);
    if (error >= 0) {
        // Save actual received bytes to an extra variable and copy rest to the given buffer
        *num_bytes_received = receive_buffer[0];
        memmove(buffer, receive_buffer + 1, receive_buffer[0]);
    } else {
        *num_bytes_received = 0;
    }

    free(receive_buffer);
    return error;
}

int ultrasonic_receiver_get_status() {
    // Just read a byte
    return i2cReadByte(set_i2c_handle);
}

void ultrasonic_receiver_wait_for_data_received() {
    // Wait until data received pin is pulled up and down
    while (gpioRead(set_data_received_pin));
    while (!gpioRead(set_data_received_pin));
}