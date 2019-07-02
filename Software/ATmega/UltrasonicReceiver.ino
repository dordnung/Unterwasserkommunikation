#include <Wire.h>

/**
 * I2C Protocol:
 *
 * First byte is mode, bytes afterwards depending on mode:
 *     MODE_RECEIVE:                1 byte how much bytes to receive (max. MAX_BYTES_PER_RECEIVE), 2 byte timeout in milliseconds (accuracy of 4ms) (0 = none)
 *                                  BYTES_RECEIVED_PIN will be pulled down and pulled up again when given bytes were received or timeout occured
 *     MODE_RECEIVE_BIT:            2 byte timeout in milliseconds (accuracy of 4ms) (0 = none)
 *                                  BYTES_RECEIVED_PIN will be pulled down and pulled up again when bit was received or timeout occured
 *     MODE_SET_FREQUENCY_LOW:      2 bytes the frequency for a LOW bit (has to be lower then the high frequency)
 *     MODE_SET_FREQUENCY_HIGH:     2 bytes the frequency for a HIGH bit (has to be higher then the low frequency)
 *     MODE_SET_FREQUENCY_BOTH:     2 bytes the frequency for a LOW bit, 2 bytes the frequency for a HIGH bit
 *     MODE_SET_DURATION_US:        2 bytes transmit duration of a bit in microseconds
 *     MODE_SET_DURATION_CYCLES:    2 bytes transmit duration of a bit in cycles
 *     MODE_SET_WAIT_TIME:          2 bytes wait time in microseconds after a bit was transmitted
 *
 * Responses after receive modes with number of (valid) received bytes (first byte) and the received bytes (0 for non valid bytes).
 *  --> Request must be done with number of requested bytes + 1
 * For other modes with STATUS_OK or STATUS_ERROR if requested.
 *
 * Do not use I2C communication while in receive mode. This will break the I2C line, as interrupts are disabled.
 */

// The slave address for I2C addressing
#define SLAVE_ADDRESS                   0x12

// Available modes
#define MODE_RECEIVE                    0x01
#define MODE_RECEIVE_BIT                0x02
#define MODE_SET_FREQUENCY_LOW          0x03
#define MODE_SET_FREQUENCY_HIGH         0x04
#define MODE_SET_FREQUENCY_BOTH         0x05
#define MODE_SET_DURATION_US            0x06
#define MODE_SET_DURATION_CYCLES        0x07
#define MODE_SET_WAIT_TIME              0x08

// Maximum bytes which can be received (-1 byte for number of (valid) received bytes)
#define MAX_BYTES_PER_RECEIVE           (BUFFER_LENGTH - 1)

// Minimum and maximum samples to analyse per bit
#define MIN_SAMPLES                     15
#define MAX_SAMPLES                     100

// Possible response status codes
#define STATUS_OK                       0x01
#define STATUS_ERROR                    0x02

// Possible meanings of duration
#define DURATION_IN_US                  0x01
#define DURATION_IN_CYCLES              0x02

// The pin on which data will be incoming
#define RECEIVE_PIN                     8

// Toggle debug pin on each edge of the received signal, debug of samples and enable Serial output
#define DEBUG_EDGE                      1
#define DEBUG_OUTPUT                    0
#define DEBUG_SAMPLES                   0
#define DEBUG_PIN                       A1
#define DEBUG_PIN_PORT                  PORTC
#define DEBUG_PIN_BIT                   PC1

#if DEBUG_OUTPUT
#define DEBUG_PRINT(x)                  Serial.print(x)
#define DEBUG_PRINTLN(x)                Serial.println(x)
#define DEBUG_FLUSH                     Serial.flush
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_FLUSH(x)
#endif

// The pin to pull down while in receiving mode
#define BYTES_RECEIVED_PIN              A0

// Default values for the settings
#define DEFAULT_FREQUENCY_LOW           29000
#define DEFAULT_FREQUENCY_HIGH          31000
#define DEFAULT_DURATION                20
#define DEFAULT_DURATION_TYPE           DURATION_IN_CYCLES
#define DEFAULT_WAIT_TIME               20000

// Time which signal must be high before receiving starts - to avoid interrupts with currently sending signal
#define SYNC_DURATION                   (wait_time * 2 / 1000)

// Additional timeout until waiting for next signal after received a bit
#if DEBUG_SAMPLES
#define EXTRA_TIMEOUT_AFTER_BIT        -(wait_time * 0.2f)
#else
#define EXTRA_TIMEOUT_AFTER_BIT        -(wait_time * 0.1f)
#endif

// Helper macros to translate frequencies and cycles to time
#define FREQUENCY_TO_MICROS(freq)       (1000000.0f / freq)
#define FREQUENCY_TO_NANOS(freq)        (1000000000L / freq)
#define CLOCK_CYCLES_TO_NANOS(cycles)   ((cycles * 125UL) / 2)

// Status of last I2C communication
volatile uint8_t status;

// Buffer to store bit samples
uint8_t sample_buffer_size;
uint16_t sample_buffer[MAX_SAMPLES];

// Buffers to store received bits and bytes
volatile uint8_t receive_buffer_size;
volatile uint8_t receive_buffer_bit;
volatile uint8_t receive_buffer[MAX_BYTES_PER_RECEIVE];

// Whether a bit or how much bytes were requested
volatile boolean requested_bit;
volatile uint8_t requested_bytes;

// 8-bit variable to check if request is set (atomic) and request status variables
volatile boolean request_set;
volatile uint16_t request_timeout;
boolean first_bit_in_request;

// The settings which are needed
volatile uint16_t frequency_low;
volatile uint16_t frequency_high;
volatile uint16_t duration;
volatile uint8_t duration_type;
volatile uint16_t wait_time;

void setup() {
    // Setup I2C
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(on_i2c_receive);
    Wire.onRequest(on_i2c_request);

    status = STATUS_OK;

    // Clear sample and receive buffer
    sample_buffer_size = 0;
    receive_buffer_size = 0;
    receive_buffer_bit = 8;

    // Clear requested status
    requested_bit = false;
    requested_bytes = 0;
    request_set = false;
    first_bit_in_request = true;
    request_timeout = 0;

    // Set the default settings
    frequency_low = DEFAULT_FREQUENCY_LOW;
    frequency_high = DEFAULT_FREQUENCY_HIGH;
    duration = DEFAULT_DURATION;
    duration_type = DEFAULT_DURATION_TYPE;
    wait_time = DEFAULT_WAIT_TIME;

    // Pull up bytes received pin
    pinMode(BYTES_RECEIVED_PIN, OUTPUT);
    digitalWrite(BYTES_RECEIVED_PIN, HIGH);

    // Set receive pin as input
    pinMode(RECEIVE_PIN, INPUT);

    // Enable debug stuff
#if DEBUG_EDGE
    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, HIGH);
#endif
#if DEBUG_OUTPUT
    Serial.begin(115200);
#endif

    // Init timer1
    init_timer1();

    DEBUG_PRINTLN("Receiver started...");
    DEBUG_FLUSH();
}

void init_timer1() {
    // To set timer, interrupts should be disabled
    uint8_t sreg = SREG;
    cli();

    // Disable timer power reduction
    PRR &= ~_BV(PRTIM1);

    // Use nothing of control register A
    TCCR1A = 0;

    // Activate input capture noise canceler and do not use prescaler
    TCCR1B = _BV(ICNC1) | _BV(CS10);

    // Restore register state again
    SREG = sreg;
}

void loop() {
    // Check if we wait for receiving something
    // Even variable can be changed in interrupt, request_set is 8-bit, so reading it is atomic
    if (request_set) {
        // Check if we are waiting for a new bit
        if ((requested_bit && receive_buffer_bit == 8) || (requested_bytes && requested_bytes > receive_buffer_size)) {
            if (first_bit_in_request) {
                // Print to debug
                DEBUG_PRINTLN("Starting to process a new request");
                debug_settings();
                DEBUG_PRINTLN("Syncing...");
                DEBUG_FLUSH();

                // Wait for sync on first bit
                wait_for_sync();
                first_bit_in_request = false;

                DEBUG_PRINTLN("Synced!");
            }

            // Print to debug
            DEBUG_PRINT("Waiting for bit ");
            DEBUG_PRINT(receive_buffer_bit);
            DEBUG_PRINT(" in byte ");
            DEBUG_PRINT(receive_buffer_size + 1);
            DEBUG_PRINT(" of ");
            DEBUG_PRINTLN((requested_bit ? 1 : requested_bytes));
            DEBUG_FLUSH();

            // Check which start state we have
            uint8_t state = digitalRead(RECEIVE_PIN);

            // Wait for a bit and remember whether there was an overflow or not
            boolean overflow = wait_for_bit(state);

            // Note when we are finished with waiting for accuracy wait
            unsigned long end = micros();

            DEBUG_PRINT("Finished waiting for bit, overflow occured: ");
            DEBUG_PRINT(overflow);
            DEBUG_PRINT(", found samples: ");
            DEBUG_PRINTLN(sample_buffer_size);
            debug_samples();
            DEBUG_FLUSH();

            // Only go on if got enough samples on overflow
            if (!overflow || sample_buffer_size >= MIN_SAMPLES) {
                // Decode the received samples to a bit
                uint32_t receive_duration;
                boolean is_high_bit;
                if (receive_sampled_bit(&receive_duration, &is_high_bit)) {
                    // Wait for the next bit by calculating when it should arrive
                    safe_delay(calculate_wait_time(overflow, end, receive_duration / 1000, is_high_bit));

                    // Go on with next loop
                    return;
                }
            }
        }

        // Otherwise we are finished -> clear variables
        request_set = false;
        first_bit_in_request = true;
        request_timeout = 0;

        // Give BYTES_RECEIVED_PIN free, so PI can request the received bytes
        digitalWrite(BYTES_RECEIVED_PIN, HIGH);

        DEBUG_PRINTLN("Finished with request!");
        DEBUG_FLUSH();
    }
}

boolean wait_for_bit(uint8_t start_state) {
    // Calculate how much overflows needed before timeout (timer overflow occurs after 0x1000 micros)
    uint16_t overflows_timeout = ceil(request_timeout / (0x1000 / 1000.0));

    // Disable interrupts to keep accuracy
    unsigned char sreg = SREG;
    cli();

    if (start_state == LOW) {
        // Enable input capture for raising edge
        TCCR1B |= _BV(ICES1);
    } else {
        // Enable input capture for falling edge
        TCCR1B &= ~_BV(ICES1);
    }

    // Set output compare register to last timer value to watch for overflows
    if (TCNT1 == 0) {
        OCR1A = 0xFFFF;
    } else {
        OCR1A = TCNT1 - 1;
    }

    // Clear timer interrupt flags
    TIFR1 = _BV(ICF1) | _BV(OCF1A);

    register uint16_t val = 0;
    register uint16_t prev = 0;
    register uint8_t tifr;

#if DEBUG_EDGE
    // Toggle debug pin on start (do not use digitalWrite, as it is slow)
    DEBUG_PIN_PORT &= ~_BV(DEBUG_PIN_BIT);
#endif

    // As long not enough samples were stored
    for (sample_buffer_size = 0; sample_buffer_size < MAX_SAMPLES;) {
#if DEBUG_EDGE
        // Toggle debug pin on new sample
        DEBUG_PIN_PORT |= _BV(DEBUG_PIN_BIT);
#endif
        // Wait for edge or overflow (output compare match)
        while (!(tifr = (TIFR1 & (_BV(ICF1) | _BV(OCF1A))))) {
        }

        // Clear timer interrupt flags for next iteration
        TIFR1 = _BV(ICF1) | _BV(OCF1A);

#if DEBUG_EDGE
        // Toggle debug pin on event
        DEBUG_PIN_PORT &= ~_BV(DEBUG_PIN_BIT);
#endif

        // Check for an overflow
        if (tifr & _BV(OCF1A)) {
            // Overflow on first edge is ok, as we want to wait for first edge, otherwise stop
            if (sample_buffer_size || (request_timeout && !(--overflows_timeout))) {
#if DEBUG_EDGE
                // Toggle debug pin on end
                DEBUG_PIN_PORT |= _BV(DEBUG_PIN_BIT);
#endif
                SREG = sreg;

                // Indicate an overflow stop by returning true
                return true;
            }

            continue;
        }

        // Save capture value to own variable to prevent multiple lookup
        val = ICR1;

        // Set new output compare register to capture value
        OCR1A = val;

        // Invert capturing edge for next capture
        TCCR1B ^= _BV(ICES1);

        // first edge has no previous value, so ignore it
        if (sample_buffer_size) {
            // Calculate the cycles since the last interrupt and save it
            if (prev < val) {
                sample_buffer[sample_buffer_size] = val - prev;
            } else {
                sample_buffer[sample_buffer_size] = 0xFFFF - prev + val;
            }
        }

        // Increase number of samples in buffer and remember previous capture value
        sample_buffer_size++;
        prev = val;
    }
#if DEBUG_EDGE
    // Toggle debug pin on end
    DEBUG_PIN_PORT |= _BV(DEBUG_PIN_BIT);
#endif

    SREG = sreg;

    // Indicate no overflow end by returning false
    return false;
}

boolean receive_sampled_bit(uint32_t *duration_nanos, boolean *is_high_bit) {
    if (duration_nanos) {
        *duration_nanos = 0;
    }

    uint32_t cycles = 0;
    uint16_t cycles_count = 0;

    // Calculate the minimum clock cycles a period of the received signal should have (to avoid reading abusive values)
    uint16_t cycles_min = (F_CPU / frequency_high) * 0.95f;
    uint16_t cycles_max = (F_CPU / frequency_low) * 1.05f;

    // Calculate the period for each HIGH and LOW of the received signal
    for (uint8_t i = 1; i < sample_buffer_size - 1; i += 2) {
        // Calculate the period of HIGH and LOW
        uint32_t cycles_period = sample_buffer[i] + sample_buffer[i + 1];

        // Check if in range
        if (cycles_period >= cycles_min && cycles_period <= cycles_max) {
            // Append to sum of cycles
            cycles += cycles_period;
            cycles_count++;
        }

        // Always append to total cycles
        if (duration_nanos) {
            *duration_nanos += cycles_period;
        }
    }

    // Calculate total time of receiving process
    if (duration_nanos) {
        *duration_nanos = CLOCK_CYCLES_TO_NANOS(*duration_nanos);
    }

    if (cycles_count >= 1) {
        // Calculate average period of received signal in nanosseconds
        int32_t total_nanos = CLOCK_CYCLES_TO_NANOS(cycles / cycles_count);

        // Convert that to a LOW or HIGH bit
        if (abs(FREQUENCY_TO_NANOS(frequency_high) - total_nanos) < abs(FREQUENCY_TO_NANOS(frequency_low) - total_nanos)) {
            *is_high_bit = true;
            DEBUG_PRINTLN("High bit found!");
            DEBUG_FLUSH();
        } else {
            *is_high_bit = false;
            DEBUG_PRINTLN("Low bit found!");
            DEBUG_FLUSH();
        }

        // Append that to the receive buffer
        bitWrite(receive_buffer[receive_buffer_size], receive_buffer_bit - 1, (*is_high_bit ? 1 : 0));

        // Go on with next bit / byte
        if (receive_buffer_bit == 1) {
            receive_buffer_bit = 8;
            receive_buffer_size++;
        } else {
            receive_buffer_bit--;
        }

        // Return true for success
        return true;
    }

    DEBUG_PRINTLN("Couldn't find a valid cycle!");
    DEBUG_FLUSH();

    return false;
}

unsigned long calculate_wait_time(boolean overflow, unsigned long end, uint32_t receive_duration, boolean is_high_bit) {
    unsigned long transmit_time;

    if (duration_type == DURATION_IN_US) {
        // If duration is in microseconds than the signal should be sent in the number of given microseconds
        transmit_time = duration;
    } else {
        // Otherwise it depends whether we received a HIGH or LOW signal
        if (is_high_bit) {
            transmit_time = FREQUENCY_TO_MICROS(frequency_high) * duration;
        } else {
            transmit_time = FREQUENCY_TO_MICROS(frequency_low) * duration;
        }
    }

    // This is how long the transmitter sends the bit and waits and some extra time
    unsigned long total_wait_time = wait_time + transmit_time + EXTRA_TIMEOUT_AFTER_BIT;

    // This it the time we took for receiving all samples and calculating which bit we received
    unsigned long already_wait_time = receive_duration + (overflow ? 0x1000 : 0) + (micros() - end);

    // Calculate wait time which left
    unsigned long wait_time_left = (total_wait_time > already_wait_time) ? (total_wait_time - already_wait_time) : 0;

    DEBUG_PRINT("Wait until next request left: ");
    DEBUG_PRINT(wait_time_left);
    DEBUG_PRINTLN(" ms");
    DEBUG_FLUSH();

    return wait_time_left;
}

void wait_for_sync() {
    unsigned long start = millis();
    uint8_t state = digitalRead(RECEIVE_PIN);

    // Wait SYNC_DURATION milliseconds for synchronisation
    while (true) {
        uint8_t prev_state = state;
        state = digitalRead(RECEIVE_PIN);

        if (prev_state == state) {
            if (millis() - start > SYNC_DURATION) {
                return;
            }
        } else {
            start = millis();
        }
    }
}

void safe_delay(unsigned long microseconds) {
    // delayMicroseconds only works until 0x3FFF (16383)
    if (microseconds > 0x3FFF) {
        unsigned long start = micros();
        while (micros() - start < microseconds);
    } else {
        delayMicroseconds(microseconds);
    }
}

void on_i2c_receive(int bytes) {
    // Set status at first to error
    status = STATUS_ERROR;

    // There must be at least two bytes, one for mode and at least one data byte
    if (bytes > 1) {
        // Get the mode and decide what to do
        uint8_t mode = Wire.read();

        switch (mode) {
            case MODE_RECEIVE:
                // Additional one byte for the number of bytes to receive and two bytes for the timeout needed
                if (bytes == 4) {
                    // Pull down BYTES_RECEIVED_PIN when receiving data
                    digitalWrite(BYTES_RECEIVED_PIN, LOW);

                    // Set atomic variable so main loop recognize that we wait for data
                    request_set = true;

                    // Clear receive buffer on start
                    receive_buffer_size = 0;
                    receive_buffer_bit = 8;

                    // First byte is the number of bytes we want
                    requested_bytes = (uint8_t)Wire.read();
                    requested_bit = false;

                    // Second and third byte is the timeout
                    request_timeout = (Wire.read() << 8) | Wire.read();

                    status = STATUS_OK;
                }

                break;
            case MODE_RECEIVE_BIT:
                // Additional two bytes for the timeout needed
                if (bytes == 3) {
                    // Pull down BYTES_RECEIVED_PIN when receiving data
                    digitalWrite(BYTES_RECEIVED_PIN, LOW);

                    // Set atomic variable so main loop recognize that we wait for data
                    request_set = true;

                    // Clear receive buffer on start
                    receive_buffer_size = 0;
                    receive_buffer_bit = 8;

                    // Indicate that we only want a single bit
                    requested_bytes = 0;
                    requested_bit = true;

                    // First and second byte is the timeout
                    request_timeout = (Wire.read() << 8) | Wire.read();

                    status = STATUS_OK;
                }

                break;
            case MODE_SET_FREQUENCY_LOW:
                // Additional two bytes for the frequency needed
                if (bytes == 3) {
                    // Just make a 16 bit value of the next two bytes
                    frequency_low = (Wire.read() << 8) | Wire.read();
                    status = STATUS_OK;
                }

                break;
            case MODE_SET_FREQUENCY_HIGH:
                // Additional two bytes for the frequency needed
                if (bytes == 3) {
                    // Just make a 16 bit value of the next two bytes
                    frequency_high = (Wire.read() << 8) | Wire.read();
                    status = STATUS_OK;
                }

                break;
            case MODE_SET_FREQUENCY_BOTH:
                // Additional four bytes for the frequencies needed
                if (bytes == 5) {
                    // Just make two 16 bit values of the next four bytes
                    frequency_low = (Wire.read() << 8) | Wire.read();
                    frequency_high = (Wire.read() << 8) | Wire.read();
                    status = STATUS_OK;
                }

                break;
            case MODE_SET_DURATION_US:
                // Additional two bytes for the duration needed
                if (bytes == 3) {
                    // Mark that duration is in microseconds and make a 16 bit value of the next two bytes
                    duration_type = DURATION_IN_US;
                    duration = (Wire.read() << 8) | Wire.read();
                    status = STATUS_OK;
                }

                break;
            case MODE_SET_DURATION_CYCLES:
                // Additional two bytes for the cycles needed
                if (bytes == 3) {
                    // Mark that duration is in cycles and make a 16 bit value of the next two bytes
                    duration_type = DURATION_IN_CYCLES;
                    duration = (Wire.read() << 8) | Wire.read();
                    status = STATUS_OK;
                }

                break;
            case MODE_SET_WAIT_TIME:
                // Additional two bytes for the wait time needed
                if (bytes == 3) {
                    // Just make a 16 bit value of the next two bytes
                    wait_time = (Wire.read() << 8) | Wire.read();
                    status = STATUS_OK;
                }

                break;
        }
    }

    // Consume all remaining bytes so next communication is not affected by them
    while (Wire.available()) {
        Wire.read();
    }
}

void on_i2c_request() {
    if (requested_bit) {
        if (receive_buffer_bit == 7) {
            // At first write the number of received bytes
            Wire.write(1);

            // Write the received byte (or bit)
            Wire.write(bitRead(receive_buffer[0], 7));
        } else {
            // At first write the number of received bytes
            Wire.write(0);

            // Write 0 on error (timeout)
            Wire.write(0);
        }

        // Received bit was transferred
        requested_bit = false;
        requested_bytes = 0;
        receive_buffer_size = 0;
        receive_buffer_bit = 8;
    } else if (requested_bytes) {
        // At first write the number of received bytes
        Wire.write(receive_buffer_size);

        for (uint8_t i = 0; i < requested_bytes; i++) {
            // Byte is valid if receive_buffer_size is greater than the current byte
            if (i < receive_buffer_size) {
                // Write valid received byte
                Wire.write(receive_buffer[i]);
            } else {
                // Write 0 on error (timeout)
                Wire.write(0);
            }
        }

        // All received bytes were transferred
        requested_bit = false;
        requested_bytes = 0;
        receive_buffer_size = 0;
        receive_buffer_bit = 8;
    } else {
        // Just response status when master is requesting it
        Wire.write(status);
    }
}

inline void debug_settings() {
    DEBUG_PRINT("Frequency Low: ");
    DEBUG_PRINT(frequency_low);
    DEBUG_PRINTLN(" Hz");

    DEBUG_PRINT("Frequency High: ");
    DEBUG_PRINT(frequency_high);
    DEBUG_PRINTLN(" Hz");

    DEBUG_PRINT("Transmit time: ");
    DEBUG_PRINT(duration);
    if (duration_type == DURATION_IN_US) {
        DEBUG_PRINTLN(" us");
    } else {
        DEBUG_PRINTLN(" cycles");
    }

    DEBUG_PRINT("Wait time: ");
    DEBUG_PRINT(wait_time);
    DEBUG_PRINTLN(" ms");
}

inline void debug_samples() {
#if DEBUG_SAMPLES
    DEBUG_PRINT(F("captured["));
    DEBUG_PRINT(sample_buffer_size);
    DEBUG_PRINT(F(" samples]="));
    DEBUG_PRINTLN();
    for (uint16_t i = 1; i < sample_buffer_size; i++) {
        uint32_t nanos = CLOCK_CYCLES_TO_NANOS(sample_buffer[i]);
        DEBUG_PRINT((i & 1) ? "-" : "+");
        DEBUG_PRINT(nanos);
        if (nanos >= 50000)
            DEBUG_PRINTLN();
        else
            DEBUG_PRINT(' ');
    }
    DEBUG_PRINTLN();
    DEBUG_PRINTLN();
#endif
}