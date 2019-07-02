#ifndef ULTRASONIC_TRANSMITTER_H
#define ULTRASONIC_TRANSMITTER_H

#include <stdint.h>

/** The possible status codes when requesting I2C status */
#define STATUS_OK     0x01
#define STATUS_ERROR  0x02

/**
 * Initialises the ultrasonic transmitter.
 * Must be called before using any other transmitter functions!
 *
 * @param transmission_end_pin The pin on which transmission end will be intimated.
 * @param slave_address The slave address of the transmitting microcontroller.
 * @param request_status Whether to request I2C status after every communication or not. Will make communication slower and unstable, so only use if needed.
 * @param i2c_bus The I2C bus of the Raspberry Pi to use. Should be 1.
 * @return 0 on success, otherwise the pigpio error number.
 */
int ultrasonic_transmitter_init(uint8_t transmission_end_pin, unsigned int slave_address, uint8_t request_status, uint8_t i2c_bus);

/**
 * Terminates the ultrasonic transmitter.
 * Must be called when transmitter will not be used anymore.
 */
void ultrasonic_transmitter_terminate();

/**
 * Sets the frequency to use for a LOW bit.
 *
 * @param frequency The frequency to use for a LOW bit.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_transmitter_set_frequency_low(uint16_t frequency);

/**
 * Sets the frequency to use for a HIGH bit.
 *
 * @param frequency The frequency to use for a HIGH bit.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_transmitter_set_frequency_high(uint16_t frequency);

/**
 * Sets the frequencies to use for a LOW and a HIGH bit.
 *
 * @param frequency_low The frequency to use for a LOW bit.
 * @param frequency_high The frequency to use for a HIGH bit.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_transmitter_set_frequencies(uint16_t frequency_low, uint16_t frequency_high);

/**
 * Sets the transmit duration in microseconds.
 *
 * @param frequency The transmit duration in microseconds.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_transmitter_set_duration_microseconds(uint16_t duration);

/**
 * Sets the transmit duration in cycles of the transmitting frequency.
 *
 * @param frequency The duration in cycles of the transmitting frequency.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_transmitter_set_duration_cycles(uint16_t duration);

/**
 * Sets the wait time in microseconds to wait between each transmitted bit.
 *
 * @param frequency The wait time in microseconds.
 * @return Status if requesting status was enabled, otherwise 10 on success or the pigpio error number on error.
 */
int ultrasonic_transmitter_set_wait_time(uint16_t wait_time);

/**
 * Transmits the least significant bit of the given byte.
 *
 * @param bit The byte which least significant bit will be transmitted.
 * @return 0 on success or the pigpio error number on error.
 */
int ultrasonic_transmit_bit(uint8_t bit);

/**
 * Transmits all given bytes. Will wait until all bytes were transmitted.
 *
 * Will be splitted into multiple transmissions if there are more then 31 bytes.
 * Use wait_time_between_transmissions in this case, so the receiver has time to receive all bytes.
 *
 * @param bytes Pointer to an array of bytes to transmit.
 * @param number_of_bytes The number of bytes to transmit.
 * @param wait_time_between_transmissions Milliseconds to wait between transmissions if transmission will be splitted. 0 for none.
 * @return 0 on success or the pigpio error number on error.
 */
int ultrasonic_transmit_bytes(const uint8_t *bytes, uint8_t number_of_bytes, uint8_t wait_time_between_transmissions);

/**
 * Transmits the given string. Will wait until all bytes were transmitted.
 *
 * Will be splitted into multiple transmissions if there are more then 31 bytes.
 * Use wait_time_between_transmissions in this case, so the receiver has time to receive all bytes.
 *
 * @param str The string to transmit. Terminating Nullbyte will not be transmitted.
 * @param wait_time_between_transmissions Milliseconds to wait between transmissions if transmission will be splitted. 0 for none.
 * @return 0 on success or the pigpio error number on error.
 */
int ultrasonic_transmit_string(const char *str, uint8_t wait_time_between_transmissions);

#endif