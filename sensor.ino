/*
 * NEED RF24 lib from https://github.com/TMRh20/RF24/
 * TODO :
 * License and cie
 * Duplicate code ?
 */
#include <SPI.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"
#include "RF24.h"

#define VCC _BV(MUX3) | _BV(MUX2) | _BV(MUX1)
#define EXTERNAL 0x01

// Time (ms) to allow the filters to settle before sending data
const int FILTER_SETTLE_TIME = 5000;

// Enable debugging on serial ?
const int DEBUG = 1;

// Speed for the serial comm
const int SERIAL_SPEED = 9600;

// Speed for the nrf module
// RF24_250KBPS / RF24_1MBPS / RF24_2MBPS
// Reduce it to improve reliability
const rf24_datarate_e NRF_SPEED = RF24_1MBPS;

// PreAmplifier level for the nRF
// Lower this to reduce power consumption. This will reduce range.
const int NRF_PA_LEVEL = RF24_PA_MAX;

// Channel for the nrf module
// 76 is default safe channel in RF24
const int NRF_CHANNEL = 76;

// Set this to 0 to enable voltage measurement.
// Else, set this to your mean voltage.
const int VOLTAGE = 240;

// Number of samples over which the mean must be done for the current measurement
const int NUMBER_SAMPLES_I = 1480;

// Pin for current measurement
const int CURRENT_PIN = A0;

// Pin for voltage measurement
const int VOLTAGE_PIN = 0;

// Calibration factor for the intensity
const double ICAL = 111.1;

// Radio pipe addresses for the 2 nodes to communicate.
// TODO : Change address
const uint64_t pipe = 0xF0F0F0F0E1LL;


// Struct to send RF data
typedef struct {
    int intensity;
    int voltage;
    int battery;
} PayloadTX;


// Next measurement to be sent
PayloadTX nrf = {0, 0, 0};

// Set up nrf24L01 radio
RF24 radio(9,10);

// Wether we are ready to send the measure or not.
// Store it because millis reinitialize after 50 days.
int settled = 0;

// Store previous value of ADCSRA
int previous_ADCSRA = 0;


// Get the battery voltage
// source is either VCC (battery) or EXTERNAL (external voltage) or a value for the MUX.
// See http://openenergymonitor.blogspot.it/2012/08/low-level-adc-control-admux.html
int readV(int mux) {
    int result = 0;
    // Use internal reference. MUX = 1110 is internal 1.1V
    ADMUX = _BV(REFS0) | mux;

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH << 8;
    result = 1126400L / result; // 1100mV / 1024 ADC steps http://openenergymonitor.org/emon/node/1186
    return result;
}

// Get the measured intensity (root mean squared)
// samples_number is the number of samples for the measurement
// http://openenergymonitor.org/emon/buildingblocks/digital-filters-for-offset-removal
// TODO : Use integer arithmetic
int readI(int samples_number) {
    int sample_I = 0;
    int last_sample_I = 0;
    int filtered_I = 0;
    int last_filtered_I = 0;
    int sum_I = 0;

    for (int n = 0; n < samples_number; n++) {
        last_sample_I = sample_I;
        sample_I = analogRead(CURRENT_PIN);
        last_filtered_I = filtered_I;
        filtered_I = 0.996 * (last_filtered_I + sample_I-last_sample_I);

        sum_I += filtered_I * filtered_I;
    }

    return ICAL * sqrt(sum_I / samples_number);
}

// Send the data through nRF
// Auto ACK is enabled so if ok is true, transmission was successful 
int sendRfData() {
    radio.powerUp();
    bool ok = radio.write(&nrf, sizeof(PayloadTX));

    if(DEBUG) {
        if(ok) {
            Serial.println("Data sent through nRF.");
        } else {
            Serial.println("Data sending failed…");
        }
    }
    radio.powerDown();

    // If unable to send data, return -1
    return (int) ok;
}

// Go to suspend for 8 seconds
// http://www.disk91.com/2014/technology/hardware/arduino-atmega328p-low-power-consumption/
void suspend() {
    // Clear various "reset" flags
    MCUSR = 0;     
    wdt_enable(WDTO_8S);
    wdt_reset();  // start watchdog timer
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // prepare for powerdown
    sleep_enable();

    previous_ADCSRA = ADCSRA;
    ADCSRA = 0; //Disable ADC
    ACSR = (1<<ACD); //Disable the analog comparator
    DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins

    power_spi_disable();
    if(DEBUG) {
        power_usart0_disable();
    }
    power_timer0_disable(); //Needed for delay and millis()
    power_timer1_disable();
    power_timer2_disable(); //Needed for asynchronous 32kHz operation

    sleep_cpu ();   // power down !
}

// Recover from suspend mode
void wakeup() {
    wdt_reset();
    power_spi_enable();
    if(DEBUG) {
        power_usart0_enable();
    }
    else {
        power_usart0_disable();
    }
    power_twi_disable();
    // Disable digital input buffer on AIN1/0
    DIDR1 = (1<<AIN1D)|(1<<AIN0D);
    disableBOD();
    // Set everything as OUTPUT LOW
    for(byte i = 0; i <= A5; i++) {
        pinMode (i, OUTPUT);
        digitalWrite (i, LOW);
    }
    power_timer0_enable();
    power_timer1_enable();
    power_timer2_enable();
    power_adc_enable();
    ADCSRA = previous_ADCSRA;
    disableBOD(); // BOD is automatically restarted at wakeup
}

// Turn off brown-out enable in software
void disableBOD() {
    set_sleep_mode(SLEEP_MODE_IDLE);  
    sleep_enable();
    MCUCR = bit (BODS) | bit (BODSE); // turn on brown-out enable select
    MCUCR = bit (BODS); // this must be done within 4 clock cycles of above
    sleep_cpu();
}

void setup() {
    // Disable non necessary functions on the arduino
    power_twi_disable();
    if(!DEBUG) {
        power_usart0_disable();
    }
    //
    // Disable digital input buffer on AIN1/0
    DIDR1 = (1<<AIN1D)|(1<<AIN0D);
    disableBOD();

    // Set everything as OUTPUT LOW
    for(byte i = 0; i <= A5; i++) {
        pinMode (i, OUTPUT);
        digitalWrite (i, LOW);
    }

    if(DEBUG) {
        Serial.begin(SERIAL_SPEED);
        Serial.println("CitizenWatt sensor.");
        Serial.println("citizenwatt.paris");
        Serial.println("Configuration:");
        Serial.println("===============");
        Serial.print("nRf speed: ");
        if(NRF_SPEED == RF24_250KBPS) Serial.print("250 kbps");
        if(NRF_SPEED == RF24_1MBPS) Serial.print("1 mbps");
        if(NRF_SPEED == RF24_2MBPS) Serial.print("2 mbps");
        Serial.println();
        Serial.print("nRf PA level: ");
        if(NRF_PA_LEVEL == RF24_PA_MIN) Serial.print("-18dBm");
        if(NRF_PA_LEVEL == RF24_PA_LOW) Serial.print("-12dBm");
        if(NRF_PA_LEVEL == RF24_PA_HIGH) Serial.print("-6dBm");
        if(NRF_PA_LEVEL == RF24_PA_MAX) Serial.print("0dBm");
        Serial.println();
        Serial.println("===============");
        Serial.println("Starting measuring.");
        Serial.println("I\tV\tBattery");
    }

    // Initialize nRF
    radio.begin();
    radio.setChannel(NRF_CHANNEL);
    // Max number of retries and max delay between them
    radio.setRetries(15, 15);
    // Reduce payload size to improve reliability
    radio.setPayloadSize(8);
    // Set the datarate
    radio.setDataRate(NRF_SPEED);
    // Use the largest CRC
    radio.setCRCLength(RF24_CRC_16);
    // Ensure auto ACK is enabled
    radio.setAutoAck(1);
    // Use the best PA level
    radio.setPALevel(NRF_PA_LEVEL);
    // Open writing pipe
    radio.openWritingPipe(pipe);

    // Enable anti-crash (restart) watchdog
    wdt_enable(WDTO_8S);
}

void loop() {
    // Check that sensor is ready
    if(!settled && millis() > FILTER_SETTLE_TIME) {
        settled = 1;
    }

    nrf.intensity = readI(NUMBER_SAMPLES_I);
    if(settled) {
        if(0 == VOLTAGE) {
            nrf.voltage = readV(EXTERNAL);
        } else {
            nrf.voltage = VOLTAGE;
        }
        nrf.battery = readV(VCC);

        if(DEBUG) {
            Serial.print(nrf.intensity);
            Serial.print("\t");
            Serial.print(nrf.voltage);
            Serial.print("\t");
            Serial.print(nrf.battery);
            Serial.print("\t");
            Serial.println();
        }

        // Send RF data
        sendRfData();

        // Go to power down mode
        suspend();
        sleep_disable();
        wakeup();
    }
}
