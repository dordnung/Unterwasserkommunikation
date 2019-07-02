#ifndef ULTRASONIC_RECEIVER_H
#define ULTRASONIC_RECEIVER_H

#include <stdint.h>

/** The possible status codes when requesting I2C status */
#define STATUS_OK     0x01
#define STATUS_ERROR  0x02

/**
 * Initialises the ultrasonic receiver with given request status and i2c bus.
 * Must be called before using any other receiver functions!
 *
 * @param data_received_pin The pin on which data receivement end will be intimated.
 * @param slave_address The slave address of the receiving microcontroller.
 * @param request_status Whether to request I2C status after every communication or not. Will make communication slower and unstable, so only use if needed.
 * @param i2c_bus The I2C bus of the Raspberry Pi to use. Should be 1.
 * @return 0 on success, otherwise the pigpio error number.
 */
int ultrasonic_receiver_init(uint8_t data_received_pin, unsigned int slave_address, uint8_t request_status, uint8_t i2c_bus);

/**
 * Terminates the ultrasonic receiver.
 * Must be called when receiver will not be used anymore.
 */
void ultrasonic_receiver_terminate();

/**
 * Sets the frequency which indicates a LOW bit.
 *
 * @param frequency The frequency which indicates a LOW bit.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_receiver_set_frequency_low(uint16_t frequency);

/**
 * Sets the frequency to use which indicates HIGH bit.
 *
 * @param frequency The frequency which indicates a HIGH bit.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_receiver_set_frequency_high(uint16_t frequency);

/**
 * Sets the frequencies which indicates a LOW or a HIGH bit.
 *
 * @param frequency_low The frequency which indicates a LOW bit.
 * @param frequency_high The frequencywhich indicates a HIGH bit.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_receiver_set_frequencies(uint16_t frequency_low, uint16_t frequency_high);

/**
 * Sets the transmit duration the transmitter uses in microseconds
 *
 * @param frequency The transmit duration in microseconds.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_receiver_set_duration_microseconds(uint16_t duration);

/**
 * Sets the transmit duration the transmitter uses in cycles of the transmitted frequency
 *
 * @param frequency The duration in cycles of the transmit frequency.
 * @return Status if requesting status was enabled, otherwise 0 on success or the pigpio error number on error.
 */
int ultrasonic_receiver_set_duration_cycles(uint16_t duration);

/**
 * Sets the wait time in microseconds the transmitter will wait between each transmitted bit.
 *
 * @param frequency The wait time in microseconds.
 * @return Status if requesting status was enabled, otherwise 10 on success or the pigpio error number on error.
 */
int ultrasonic_receiver_set_wait_time(uint16_t wait_time);

/**
 * Receives a single bit.
 *
 * @param bit The bit to receive to.
 * @param timeout Timeout for receiving in milliseconds. 0 for none.
 * @return 0 on success or the pigpio error number on error.
 */
int ultrasonic_receive_bit(uint8_t *bit, uint16_t timeout);

/**
 * Receives a number of bytes. Will wait until the number of bytes received or timed out.
 * Be sure to add extra wait time at the transmitter if receiving more then 31 bytes, so there is enough time to transfer all bytes.
 *
 * @param bytes Array to save received bytes to.
 * @param number_of_bytes The number of bytes to receive.
 * @param timeout Timeout for receiving in milliseconds. 0 for none.
 * @param bytes_received The number of actual received bytes. Can't be null!
 * @return 0 on success or the pigpio error number on error.
 */
int ultrasonic_receive_bytes(uint8_t *bytes, uint8_t number_of_bytes, uint16_t timeout, uint8_t *bytes_received);

/**
 * Receives a string with given length. Will wait until all bytes of the string received or timed out.
 * Be sure to add extra wait time at the transmitter if receiving string with more then 31 bytes, so there is enough time to transfer all bytes.
 *
 * @param str Char array to save string to. It must have at least a size of (str_length + 1) bytes.
 * @param str_length The length of the string to receive.
 * @param timeout Timeout for receiving in milliseconds. 0 for none.
 * @param bytes_received The number of actual received bytes. Can't be null!
 * @return 0 on success or the pigpio error number on error.
 */
int ultrasonic_receive_string(char *str, uint8_t str_length, uint16_t timeout, uint8_t *bytes_received);

#endif