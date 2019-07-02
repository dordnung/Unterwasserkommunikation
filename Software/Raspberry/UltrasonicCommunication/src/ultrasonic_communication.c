#include "ultrasonic_communication.h"
#include <pigpio.h>

// Store if we already initialised
static uint8_t is_library_init = 0;

int ultrasonic_communication_library_init() {
    if (!is_library_init) {
        // Initialise the pigpio library and check result
        int gpio_init = gpioInitialise();
        if (gpio_init < 0) {
            return gpio_init;
        }
    }

    is_library_init = 1;
    return 0;
}

int is_ultrasonic_communication_library_init() {
    return is_library_init;
}

void ultrasonic_communication_library_terminate() {
    // Terminate the pigpio
    gpioTerminate();

    is_library_init = 0;
}