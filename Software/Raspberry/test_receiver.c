#include <ultrasonic_communication.h>
#include <ultrasonic_receiver.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RECEIVER_ADDRESS   0x12
#define DATA_RECEIVED_PIN  22
#define FREQ_LOW           29000
#define FREQ_HIGH          31000

void receive_1_test(int timeout) {
    uint8_t byte;
    uint8_t received;

    int error;
    if ((error = ultrasonic_receive_bytes(&byte, 1, timeout, &received)) >= 0) {
        printf("Received %d byte:\n", received);
        printf("\t%d\n", byte);
    } else {
        printf("Couldn't receive 1 byte, error: %d\n", error);
    }
}

void receive_8_test(int timeout) {
    uint8_t bytes[8];
    uint8_t received;

    int error;
    if ((error = ultrasonic_receive_bytes(bytes, 8, timeout, &received)) >= 0) {
        printf("Received %d bytes:\n", received);
        for (uint8_t i = 0; i < received; i++) {
            printf("\t%d\n", bytes[i]);
        }
    } else {
        printf("Couldn't receive 8 bytes, error: %d\n", error);
    }
}

void receive_x_test(unsigned int x, int timeout) {
    uint8_t *bytes = (uint8_t *)malloc(x * sizeof(uint8_t));
    uint8_t received;

    int error;
    if ((error = ultrasonic_receive_bytes(bytes, x, timeout, &received)) >= 0) {
        printf("Received %d bytes:\n", received);
        for (uint8_t i = 0; i < received; i++) {
            printf("\t%d\n", bytes[i]);
        }
    } else {
        printf("Couldn't receive %d bytes, error: %d\n", x, error);
    }
    
    free(bytes);
}

void receive_hello_test(int timeout) {
    char str[7];
    uint8_t received;

    int error;
    if ((error = ultrasonic_receive_string(str, strlen("hello"), timeout, &received)) >= 0) {
        printf("Received %d bytes and string: %s\n", received, str);
    } else {
        printf("Couldn't receive hello, error: %d\n", error);
    }
}

void receive_bit_test(int timeout) {
    uint8_t bit;

    int error;
    if ((error = ultrasonic_receive_bit(&bit, timeout)) >= 0) {
        printf("Received bit: %d\n", bit);
    } else {
        printf("Couldn't receive bit, error: %d\n", error);
    }
}

void evaluate_test(int timeout) {
    uint8_t expected[8] = { 170, 255, 0, 73, 240, 15, 85, 1 };
    uint8_t bytes[8];
    uint8_t received;
    uint8_t current = 0;
    
    //100 times 8 byte
    while (current < 100) {
        int error;
        if ((error = ultrasonic_receive_bytes(bytes, 8, timeout, &received)) >= 0) {
            if (received != 8) {
                printf("Expected 8 bytes, got %d\n", received);
                return;
            }
            
            for (uint8_t i = 0; i < received; i++) {
                if (bytes[i] != expected[i]) {
                    printf("Got an invalid bit. Expected %d, got %d\n", expected[i], bytes[i]);
                    return;
                }
            }
            
            current++;
            printf("Successfull received: %d\n", current);
        } else {
            printf("Couldn't receive 8 bytes, error: %d\n", error);
            return;
        }
    }
    
    // 5 times 200 bytes
    uint8_t bytes2[200];
    current = 0;
    
    while (current < 5) {
        int error;
        if ((error = ultrasonic_receive_bytes(bytes2, 200, timeout, &received)) >= 0) {
            if (received != 200) {
                printf("Expected 200 bytes, got %d\n", received);
                break;
            }
            
            for (uint8_t i = 0; i < received; i++) {
                if (bytes2[i] != i) {
                    printf("Got an invalid bit. Expected %d, got %d\n", i, bytes2[i]);
                    return;
                }
            }
            
            current++;
            printf("Successfull received 200 bytes: %d\n", current);
        } else {
            printf("Couldn't receive 200 bytes, error: %d\n", error);
            break;
        }
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: test_receiver <test> [wait_time] [cycles] [freq_low] [freq_high] [timeout]\n");
        printf("Tests:\n");
        printf("\t1: Receive 1 Byte\n");
        printf("\t2: Receive 8 Byte\n");
        printf("\t3: Receive 100 Byte\n");
        printf("\t4: Receive Bit\n");
        printf("\t5: Receive Bit\n");
        printf("\t6: Receive Hello\n");
        printf("\t7: Receive 100 x 8 + 5 x 200 Byte\n");
        
        exit(1);
    }
    
    printf("Init library: %d\n", ultrasonic_communication_library_init());
    printf("Init receiver: %d\n", ultrasonic_receiver_init(DATA_RECEIVED_PIN, RECEIVER_ADDRESS, 0, 1));

    if (argc >= 3) {
        printf("Set wait time %d: %d\n", atoi(argv[2]), ultrasonic_receiver_set_wait_time(atoi(argv[2])));
    } else {
        printf("Set wait time 20000: %d\n", ultrasonic_receiver_set_wait_time(20000));
    }
    
    if (argc >= 4) {
        printf("Set duration %d: %d\n", atoi(argv[3]), ultrasonic_receiver_set_duration_cycles(atoi(argv[3])));
    } else {
        printf("Set duration 20: %d\n", ultrasonic_receiver_set_duration_cycles(20));
    }
    
    if (argc >= 5) {
        printf("Set freq low %d: %d\n", atoi(argv[4]), ultrasonic_receiver_set_frequency_low(atoi(argv[4])));
    } else {
        printf("Set freq low %d: %d\n", FREQ_LOW, ultrasonic_receiver_set_frequency_low(FREQ_LOW));
    }
    
    if (argc >= 6) {
        printf("Set freq low %d: %d\n", atoi(argv[5]), ultrasonic_receiver_set_frequency_high(atoi(argv[5])));
    } else {
        printf("Set freq low %d: %d\n", FREQ_HIGH, ultrasonic_receiver_set_frequency_high(FREQ_HIGH));
    }
    
    int timeout = 10000;
    if (argc >= 7) {
        timeout = atoi(argv[6]);
    }
    printf("Timeout %d\n", timeout);
    
    int test = atoi(argv[1]);
    if (test == 1) {
        printf("Starting 1 Byte test...\n");
        receive_1_test(timeout);
    } else if (test == 2) {
        printf("Starting 8 Byte test...\n");
        receive_8_test(timeout);
    } else if (test == 3) {
        printf("Starting 100 Byte test...\n");
        receive_x_test(100, timeout);
    } else if (test == 4) {
        printf("Starting Bit test...\n");
        receive_bit_test(timeout);
    } else if (test == 5) {
        printf("Starting Bit test...\n");
        receive_bit_test(timeout);
    } else if (test == 6) {
        printf("Starting Hello test...\n");
        receive_hello_test(timeout);
    } else if (test == 7) {
        printf("Starting Evaluate test...\n");
        evaluate_test(timeout);
    }

    ultrasonic_receiver_terminate();
    ultrasonic_communication_library_terminate();
}