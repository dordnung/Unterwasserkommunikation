#ifndef ULTRASONIC_COMMUNICATION_H
#define ULTRASONIC_COMMUNICATION_H

#include <stdint.h>

/**
 * Initialises the ultrasonic communication library by initialising the pigpio library.
 * Must be called before using any other library functions!
 *
 * @return 0 on success, otherwise the pigpio error number.
 */
int ultrasonic_communication_library_init();

/**
 * Returns whether the ultrasonic communication library is initialised.
 *
 * @return 1 if initialised, otherwise 0.
 */
int is_ultrasonic_communication_library_init();

/**
 * Terminates the ultrasonic communication library by terminating the pigpio library.
 * Must be called on program exit!
 */
void ultrasonic_communication_library_terminate();

#endif