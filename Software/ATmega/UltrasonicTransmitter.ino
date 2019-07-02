#include <Wire.h>

/**
 * I2C Protocol:
 *
 * First byte is mode, bytes afterwards depending on mode:
 *     MODE_TRANSMIT:             x bytes data to transmit, TRANSMISSION_END_PIN will be pulled down and pulled up again when x bytes were transmitted
 *     MODE_TRANSMIT_BIT:         1 byte data (LSF bit will be transmitted), TRANSMISSION_END_PIN will be pulled down and pulled up when bit was transmitted
 *     MODE_SET_FREQUENCY_LOW:    2 bytes the frequency for a LOW bit (has to be lower then the high frequency)
 *     MODE_SET_FREQUENCY_HIGH:   2 bytes the frequency for a HIGH bit (has to be higher then the low frequency)
 *     MODE_SET_FREQUENCY_BOTH:   2 bytes the frequency for a LOW bit, 2 bytes the frequency for a HIGH bit
 *     MODE_SET_DURATION_US:      2 bytes transmit duration of a bit in microseconds
 *     MODE_SET_DURATION_CYCLES:  2 bytes transmit duration of a bit in cycles
 *     MODE_SET_WAIT_TIME:        2 bytes wait time in microseconds after a bit was transmitted
 *
 * Responses with STATUS_OK or STATUS_ERROR if requested (use TRANSMISSION_END_PIN when transmitting instead of requesting status).
 *
 * Do not use I2C communication while in transmit mode. This will break the transmit timing because of interrupts.
 */

// The slave address for I2C addressing
#define SLAVE_ADDRESS            0x11

// Available modes
#define MODE_TRANSMIT            0x01
#define MODE_TRANSMIT_BIT        0x02
#define MODE_SET_FREQUENCY_LOW   0x03
#define MODE_SET_FREQUENCY_HIGH  0x04
#define MODE_SET_FREQUENCY_BOTH  0x05
#define MODE_SET_DURATION_US     0x06
#define MODE_SET_DURATION_CYCLES 0x07
#define MODE_SET_WAIT_TIME       0x08

// Maximum bytes which can be transmitted (-1 byte for mode selection)
#define MAX_BYTES_PER_TRANSMIT   (BUFFER_LENGTH - 1)

// Possible response status codes
#define STATUS_OK                0x01
#define STATUS_ERROR             0x02

// Possible meanings of duration
#define DURATION_IN_US           0x01
#define DURATION_IN_CYCLES       0x02

// The pin to pull down while transmitting
#define TRANSMISSION_END_PIN     2

// The pin to activate the line driver before transmission
#define ACTIVATE_DRIVER_PIN      A3

// The time the line driver needs until it's fully activated in microseconds
#define DRIVER_ACTIVATION_TIME   300

// Default values for the settings
#define DEFAULT_FREQUENCY_LOW    29000
#define DEFAULT_FREQUENCY_HIGH   31000
#define DEFAULT_DURATION         20
#define DEFAULT_DURATION_TYPE    DURATION_IN_CYCLES
#define DEFAULT_WAIT_TIME        20000

// Status of last I2C communication
volatile uint8_t status;

// Buffers for transmitting
volatile boolean transmit_single_bit;
volatile uint8_t transmit_buffer_size;
volatile uint8_t transmit_buffer[MAX_BYTES_PER_TRANSMIT];

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

    // Clear transmit buffer
    transmit_single_bit = false;
    transmit_buffer_size = 0;

    // Set the default settings
    frequency_low = DEFAULT_FREQUENCY_LOW;
    frequency_high = DEFAULT_FREQUENCY_HIGH;
    duration = DEFAULT_DURATION;
    duration_type = DEFAULT_DURATION_TYPE;
    wait_time = DEFAULT_WAIT_TIME;

    // Pull up transmission end pin
    pinMode(TRANSMISSION_END_PIN, OUTPUT);
    digitalWrite(TRANSMISSION_END_PIN, HIGH);

    // Disable line driver on begin
    pinMode(ACTIVATE_DRIVER_PIN, OUTPUT);
    digitalWrite(ACTIVATE_DRIVER_PIN, HIGH);

    // Init timer1
    init_timer1();
}

void init_timer1() {
    // To set timer, interrupts should be disabled
    uint8_t sreg = SREG;
    cli();

    // Set timer pins to output (DDB1 and DDB2 for timer 1)
    DDRB |= (_BV(PB1) | _BV(PB2));

    // Disable timer power reduction
    PRR &= ~_BV(PRTIM1);

    // Disable timer 1 to begin
    TCCR1A = 0;
    TCCR1B = 0;

    // Restore register state again
    SREG = sreg;
}

void loop() {
    // Check if we need to transmit something
    // Even variable can be changed in interrupt, transmit_buffer_size is 8-bit, so reading it is atomic
    if (transmit_buffer_size) {
        // Activate the line driver before transmitting
        digitalWrite(ACTIVATE_DRIVER_PIN, LOW);
        delayMicroseconds(DRIVER_ACTIVATION_TIME);

        if (transmit_single_bit) {
            // Only transmit the LSF bit in the first byte of the transmit buffer
            transmit_bit(transmit_buffer[0]);
        } else {
            // Transmit all bytes in the transmit buffer
            for (uint8_t i = 0; i < transmit_buffer_size; i++) {
                transmit_byte(transmit_buffer[i]);
            }
        }

        // Finished transmitting -> reset transmit buffer status
        transmit_single_bit = false;
        transmit_buffer_size = 0;

        // Pull up TRANSMISSION_END_PIN to indicate that transmission is finished
        digitalWrite(TRANSMISSION_END_PIN, HIGH);

        // Disable the line driver after transmitting
        digitalWrite(ACTIVATE_DRIVER_PIN, HIGH);
    }
}

void transmit_bit(uint8_t byte) {
    // Just transmit the LSF bit of the byte
    transmit((byte & 0x01) ? frequency_high : frequency_low);

    // Wait the given wait time after transmission of a bit
    safe_delay(wait_time);
}

void transmit_byte(uint8_t byte) {
    // Transmit all bits of the byte
    for (uint8_t i = 0; i < 8; i++) {
        // Transmit MSF bit first
        transmit(bitRead(byte, 7 - i) ? frequency_high : frequency_low);

        // Wait the given wait time after transmission of a bit
        safe_delay(wait_time);
    }
}

void transmit(uint16_t frequency) {
    // To set timer, interrupts should be disabled
    uint8_t sreg = SREG;
    cli();

    // Calcuate top of the timer
    unsigned long top = F_CPU / frequency / 2 - 1;

    // For used PWM phase and frequency mode the top has to be written to ICR1
    ICR1 = top;
    if (TCNT1 > top) {
        TCNT1 = top;
    }

    // Set PWM duty cycle to 50%
    OCR1A = OCR1B = top / 2;

    // Use non inverting PWM on OC1A pin and inverting PWM on OC1B
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0);

    // Start phase and frequency corrected PWM with prescaler 1
    TCCR1B = _BV(WGM13) | _BV(CS10);

    // Wait given duration
    if (duration_type == DURATION_IN_CYCLES) {
        // Transmit as long as the given cycles of this frequency
        delayMicroseconds((1000000.0f / frequency) * duration);
    } else {
        // Transmit as long as the given microseconds
        delayMicroseconds(duration);
    }

    // Disable timer when finished
    TCCR1B = 0;
    TCCR1A = 0;

    // Set the timer pins to LOW
    PORTB &= ~(_BV(PB1) | _BV(PB2));

    // Restore register state again
    SREG = sreg;
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
            case MODE_TRANSMIT:
                // Pull down TRANSMISSION_END_PIN when transmitting data
                digitalWrite(TRANSMISSION_END_PIN, LOW);

                // Read all bytes and copy them into the transmit buffer
                while (Wire.available()) {
                    transmit_buffer[transmit_buffer_size++] = (uint8_t)Wire.read();
                }

                status = STATUS_OK;
                break;
            case MODE_TRANSMIT_BIT:
                // Only one byte allowed when transmitting a bit
                if (bytes == 2) {
                    // Pull down TRANSMISSION_END_PIN when transmitting data
                    digitalWrite(TRANSMISSION_END_PIN, LOW);

                    // Copy the byte into the transmit buffer and indicate that we only want to transmit the LSF bit
                    transmit_buffer[0] = (uint8_t)Wire.read();
                    transmit_buffer_size = 1;
                    transmit_single_bit = true;

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
    // Just respond with latest status when master is requesting it
    Wire.write(status);
}