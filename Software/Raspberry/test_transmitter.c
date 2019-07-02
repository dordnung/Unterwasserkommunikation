#include <ultrasonic_communication.h>
#include <ultrasonic_transmitter.h>

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

#define TRANSMITTER_ADDRESS   0x11
#define TRANSMISSION_END_PIN  26
#define FREQ_LOW              29000
#define FREQ_HIGH             31000

void frequency_test(int cycles) {
    printf("Set duration: %d\n", ultrasonic_transmitter_set_duration_cycles(cycles));
    printf("Set wait time: %d\n", ultrasonic_transmitter_set_wait_time(50000));

    for (int i = 25000; i < 50000; i += 1000) {
        printf("Set frequency high (%d): %d\n", i, ultrasonic_transmitter_set_frequency_high(i));
        ultrasonic_transmit_bit(1);
    }
}

void transmit_1_test() {
    uint8_t byte = 1;
    printf("Transmitted 1 byte: %d\n", ultrasonic_transmit_bytes(&byte, 1, 0));
}

void transmit_8_test() {
    uint8_t bytes[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    printf("Transmitted 8 bytes: %d\n", ultrasonic_transmit_bytes(bytes, 8, 0));
}

void transmit_x_test(unsigned int x, int timeout) {
    uint8_t *bytes = (uint8_t *)malloc(x * sizeof(uint8_t));
    for (int i = 0; i < x; i++) {
        bytes[i] = i;
    }

    printf("Transmitted %d bytes: %d\n", x, ultrasonic_transmit_bytes(bytes, x, timeout));
    free(bytes);
}

void transmit_hello_test(int timeout) {
    printf("Transmitted hello: %d\n", ultrasonic_transmit_string("hello", timeout));
}

void transmit_bit_high_test() {
    printf("Transmitted high bit: %d\n", ultrasonic_transmit_bit(1));
}

void transmit_bit_low_test() {
    printf("Transmitted low bit: %d\n", ultrasonic_transmit_bit(0));
}

void evaluate_test(int timeout) {
    uint8_t bytes[8] = { 170, 255, 0, 73, 240, 15, 85, 1 };
    
    // 100 times 8 bytes
    for (int i=0; i < 100; i++) {
        printf("Transmitted 8 bytes: %d\n", ultrasonic_transmit_bytes(bytes, 8, 0));
        gpioSleep(PI_TIME_RELATIVE, 0, 50000);
    }
    
    // 5 times 200 byte
    transmit_x_test(200, timeout);
    gpioSleep(PI_TIME_RELATIVE, 0, 50000);
    transmit_x_test(200, timeout);
    gpioSleep(PI_TIME_RELATIVE, 0, 50000);
    transmit_x_test(200, timeout);
    gpioSleep(PI_TIME_RELATIVE, 0, 50000);
    transmit_x_test(200, timeout);
    gpioSleep(PI_TIME_RELATIVE, 0, 50000);
    transmit_x_test(200, timeout);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: test_transmitter <test> [wait_time] [cycles] [freq_low] [freq_high] [timeout]\n");
        printf("Tests:\n");
        printf("\t0: Frequency test\n");
        printf("\t1: Receive 1 Byte\n");
        printf("\t2: Receive 8 Byte\n");
        printf("\t3: Receive 100 Byte\n");
        printf("\t4: Receive High Bit\n");
        printf("\t5: Receive Low Bit\n");
        printf("\t6: Receive Hello\n");
        printf("\t7: Receive 100 x 8 + 5 x 200 Byte\n");
        
        exit(1);
    }
    
    printf("Init library: %d\n", ultrasonic_communication_library_init());
    printf("Init transmitter: %d\n", ultrasonic_transmitter_init(TRANSMISSION_END_PIN, TRANSMITTER_ADDRESS, 0, 1));

    if (argc >= 3) {
        printf("Set wait time %d: %d\n", atoi(argv[2]), ultrasonic_transmitter_set_wait_time(atoi(argv[2])));
    } else {
        printf("Set wait time 20000: %d\n", ultrasonic_transmitter_set_wait_time(20000));
    }
    
    int cycles = 20;
    if (argc >= 4) {
        cycles = atoi(argv[3]);
    }
    printf("Set duration %d: %d\n", cycles, ultrasonic_transmitter_set_duration_cycles(cycles));
    
    if (argc >= 5) {
        printf("Set freq low %d: %d\n", atoi(argv[4]), ultrasonic_transmitter_set_frequency_low(atoi(argv[4])));
    } else {
        printf("Set freq low %d: %d\n", FREQ_LOW, ultrasonic_transmitter_set_frequency_low(FREQ_LOW));
    }
    
    if (argc >= 6) {
        printf("Set freq low %d: %d\n", atoi(argv[5]), ultrasonic_transmitter_set_frequency_high(atoi(argv[5])));
    } else {
        printf("Set freq low %d: %d\n", FREQ_HIGH, ultrasonic_transmitter_set_frequency_high(FREQ_HIGH));
    }
    
    int timeout = 100;
    if (argc >= 7) {
        timeout = atoi(argv[6]);
    }
    printf("Timeout %d\n", timeout);
    
    int test = atoi(argv[1]);
    if (test == 0) {
        printf("Starting frequency test...\n");
        frequency_test(cycles);
    } else if (test == 1) {
        printf("Starting 1 Byte test...\n");
        transmit_1_test();
    } else if (test == 2) {
        printf("Starting 8 Byte test...\n");
        transmit_8_test();
    } else if (test == 3) {
        printf("Starting 100 Byte test...\n");
        transmit_x_test(100, timeout);
    } else if (test == 4) {
        printf("Starting HIGH Bit test...\n");
        transmit_bit_high_test();
    } else if (test == 5) {
        printf("Starting LOW Bit test...\n");
        transmit_bit_low_test();
    } else if (test == 6) {
        printf("Starting Hello test...\n");
        transmit_hello_test(timeout);
    } else if (test == 7) {
        printf("Starting Evaluate test...\n");
        evaluate_test(timeout);
    }

    ultrasonic_transmitter_terminate();
    ultrasonic_communication_library_terminate();
}