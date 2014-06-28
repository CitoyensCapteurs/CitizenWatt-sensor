/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 TMRh20 2014 - Updates to the library allow sleeping both in TX and RX modes:
      TX Mode: The radio can be powered down (.9uA current) and the Arduino slept using the watchdog timer
      RX Mode: The radio can be left in standby mode (22uA current) and the Arduino slept using an interrupt pin
 */

/**
 * Example RF Radio Ping Pair which Sleeps between Sends
 *
 * This is an example of how to use the RF24 class to create a battery-
 * efficient system.  It is just like the GettingStarted_CallResponse example, but the
 * ping node powers down the radio and sleeps the MCU after every
 * ping/pong cycle, and the receiver sleeps between payloads.
 *
 * Write this sketch to two different nodes,
 * connect the role_pin to ground on one.  The ping node sends the current
 * time to the pong node, which responds by sending the value back.  The ping
 * node can then see how long the whole cycle took.
 */

#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "EmonLib.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define VCC _BV(MUX3) | _BV(MUX2) | _BV(MUX1)
#define EXTERNAL 0x01

/***************************************************************
 *      CHANGE BELOW THIS LINE TO EDIT CONFIG
****************************************************************/

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
const rf24_pa_dbm_e NRF_PA_LEVEL = RF24_PA_MAX;

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


// Struct to send RF data
typedef struct {
    int power;
} PayloadTX;

// Next measurement to be sent
PayloadTX nrf = {0};

EnergyMonitor ct1;


// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };   // Radio pipe addresses for the 2 nodes to communicate.

// Sleep declarations
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;

void setup_watchdog(uint8_t prescalar);
void do_sleep(void);

const short sleep_cycles_per_transmission = 4;
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;

void setup(){
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
  Serial.println("I\tV\tPower");

  // Prepare sleep parameters
  // Only the ping out role uses WDT.  Wake up every 4s to send a ping
  //if ( role == role_ping_out )
    //setup_watchdog(wdt_4s);

  // Setup and configure rf radio

  radio.begin();
  radio.setRetries(15,15);

  // Open pipes to other nodes for communication

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

    radio.openWritingPipe(pipes[0]);
    //radio.openReadingPipe(1,pipes[1]);

  // Start listening
  radio.startListening();

  // Dump the configuration of the rf unit for debugging
  //radio.printDetails();
  
  ct1.currentTX(0, ICAL);
}

void loop(){
        nrf.power = ct1.calcIrms(1480) * 240.0;

        Serial.print(nrf.power);
        Serial.print("\t");
        Serial.println();
        
  
    //radio.powerUp();                                // Power up the radio after sleeping
    radio.stopListening();                          // First, stop listening so we can talk.
                         
    Serial.print("Now sending... \n\r");
    
    bool ok = radio.write(&nrf, sizeof(PayloadTX));
    
   /* unsigned long started_waiting_at = millis();    // Wait here until we get a response, or timeout (250ms)
    bool timeout = false;
    while ( ! radio.available()  ){
        if (millis() - started_waiting_at > 250 ){  // Break out of the while loop if nothing available
          timeout = true;
          break;
        }
    }
    
    if ( timeout ) {                                // Describe the results
        Serial.print("Failed, response timed out.\n\r");
    }*/
    // Shut down the system
    delay(500);                     // Experiment with some delay here to see if it has an effect
                                    // Power down the radio.  
    //radio.powerDown();              // NOTE: The radio MUST be powered back up again manually

                                    // Sleep the MCU.
     // do_sleep();


}


/*void wakeUp(){
  sleep_disable();
}

// Sleep helpers

//Prescaler values
// 0=16ms, 1=32ms,2=64ms,3=125ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec

void setup_watchdog(uint8_t prescalar){

  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
}

ISR(WDT_vect)
{
  //--sleep_cycles_remaining;
  Serial.println("WDT");
}

void do_sleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  attachInterrupt(0,wakeUp,LOW);
  WDTCSR |= _BV(WDIE);
  sleep_mode();                        // System sleeps here
                                       // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out  
  detachInterrupt(0);  
  WDTCSR &= ~_BV(WDIE);  
}*/

/* Returns the measured intensity (root mean squared)
 *
 * params : samples_number is the number of samples used for averaging
 * See http://openenergymonitor.org/emon/buildingblocks/digital-filters-for-offset-removal
 *
 * TODO: Use integer arithmetic
 */


/* Returns the battery voltage
 * 
 * params : source is either VCC (battery) or EXTERNAL (external voltage) or a value for the MUX register
 * See http://openenergymonitor.blogspot.it/2012/08/low-level-adc-control-admux.html
 */
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

