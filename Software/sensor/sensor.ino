 ///////////////////
//   Includes    //
///////////////////

#include <EEPROM.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "EmonLib.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

///////////////////////
//   CONFIGURATION   //
///////////////////////

#define DEBUG 1

// input is for reading serial inputs, inMenu for menu navigation
char input;
int inMenu = 0;

#define SERIAL_BAUDRATE 57600

// Speed for the nrf module
// RF24_250KBPS / RF24_1MBPS / RF24_2MBPS
// Reduce it to improve reliability
rf24_datarate_e NRF_SPEED;
#define NRF_SPEED_EEPROM 0

// PreAmplifier level for the nRF
// Lower this to reduce power consumption. This will reduce range.
rf24_pa_dbm_e NRF_PA_LEVEL;
#define NRF_PA_LEVEL_EEPROM 1

// Channel for the nrf module
// 76 is default safe channel in RF24
int NRF_CHANNEL;
#define NRF_CHANNEL_EEPROM 2

// Set this to 0 to enable voltage measurement.
// Else, set this to your mean voltage.
int VOLTAGE;
#define VOLTAGE_EEPROM 3

// Number of samples over which the mean must be done for the current measurement
int NUMBER_SAMPLES_I;
#define NUMBER_SAMPLES_I_EEPROM 4

// Pin for current measurement
const int CURRENT_PIN = A0;

// Pin for voltage measurement
const int VOLTAGE_PIN = 0;

// Pin for the green LED
const int GREEN_LED_PIN = 2;

// Pin for the red LED
const int RED_LED_PIN = 3;

// Energy Monitor object
EnergyMonitor emon1;

// Calibration factor for the intensity
double ICAL = 90.9;
#define ICAL_EEPROM 5

// Radio pipe addresses for the 2 nodes to communicate.
uint64_t pipes[2] = { 0xE056D446D0LL, 0xF0F0F0F0D2LL };

#define PIPE_EEPROM 6

///////////////////////
//   Declarations    //
///////////////////////

#define TIMEOUT 250 // timeout in ms

// Struct to send RF data
typedef struct {
    int power;
    int voltage;
    int battery;
} PayloadTX;

// Next measurement to be sent
PayloadTX nrf = {0, 0, 0};

////////////////////////////////
//   Hardware configuration   //
////////////////////////////////

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

//////////////////////////////
//   Sleep configuration    //
//////////////////////////////

typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;

void setup_watchdog(uint8_t prescalar);
void do_sleep(void);

//////////////////////////
//   Setup operation    //
//////////////////////////

void setup(void)
{
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
  
  
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println(F("/!\\ STARTING CitizenWatt Sensor"));
  
  printf_begin();

  // Read the config stored on EEPROM
  int nrf_speed_eeprom = EEPROM.read(NRF_SPEED_EEPROM);
  int nrf_pa_level_eeprom = EEPROM.read(NRF_PA_LEVEL_EEPROM);
  int nrf_channel_eeprom = EEPROM.read(NRF_CHANNEL_EEPROM);
  int voltage_eeprom = EEPROM.read(VOLTAGE_EEPROM);
  int number_samples_i_eeprom = EEPROM.read(NUMBER_SAMPLES_I_EEPROM);
  int ical_eeprom = EEPROM.read(ICAL_EEPROM);
  int pipe_eeprom = EEPROM.read(PIPE_EEPROM);
  delay(1000);

  Serial.println(F("//////////////////////////////"));
  Serial.println(F("//    CitizenWatt sensor    //"));
  Serial.println(F("//    citizenwatt.paris     //"));
  Serial.println(F("////////////////////////////// \n"));
  Serial.println(F("Configuration:"));
  Serial.println(F("=============================="));

  // Load the config
  load(nrf_speed_eeprom, nrf_pa_level_eeprom, nrf_channel_eeprom,
       voltage_eeprom, number_samples_i_eeprom, ical_eeprom,
       pipe_eeprom);

  Serial.println();
  Serial.println(F("[+] Go to the config menu ? y/[n] - autostart in 10s"));
  inMenu = 1;

  //
  // Prepare sleep parameters
  //

  setup_watchdog(wdt_8s); // /!\ 8s sleeping

  //
  // Setup and configure rf radio
  //

  // Initialize nRF
  radio.begin();
  radio.setChannel(0x4c);//NRF_CHANNEL);

  // Max number of retries and max delay between them
  radio.setRetries(15, 15);

  // Reduce payload size to improve reliability
  radio.setPayloadSize(8);

  // Set the datarate
  radio.setDataRate(NRF_SPEED);

  // Use the largest CRC
  radio.setCRCLength(RF24_CRC_16);

  // Ensure auto ACK is enabled
  //radio.setAutoAck(1);

  // Use the adapted PA level
  radio.setPALevel(NRF_PA_LEVEL);

  // Open writing pipe
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);

  if(DEBUG) {
    radio.printDetails();
  }

  // Start listening
  //radio.startListening();
  
  // Init EnergyMonitor, with calibration factor for R1 = 22 Ohms
  emon1.current(0, 90.9);

}

void load(int nrf_speed_eeprom, int nrf_pa_level_eeprom, int nrf_channel_eeprom, int voltage_eeprom,
         int number_samples_i_eeprom, int ical_eeprom, int pipe_eeprom)
{
  switch(nrf_speed_eeprom) {
    case '1':
      NRF_SPEED = RF24_250KBPS;
      Serial.println(F("[+] Successfully set nRF speed to 250 kbps"));
      break;
    case '2':
      NRF_SPEED = RF24_1MBPS;
      Serial.println(F("[+] Successfully set nRF speed to 1 mbps"));
      break;
    case '3':
      Serial.println(F("[+] Successfully set nRF speed to 2 mbps"));
      NRF_SPEED = RF24_2MBPS;
      break;
  }
  switch(nrf_pa_level_eeprom) {
    case '1':
      Serial.println(F("[+] Successfully set nRF power to -18 dBm"));
      NRF_PA_LEVEL = RF24_PA_MIN;
      break;
    case '2':
      NRF_PA_LEVEL = RF24_PA_LOW;
      Serial.println(F("[+] Successfully set nRF power to -12 dBm"));
      break;
    case '3':
      //NRF_PA_LEVEL = RF24_PA_MED;
      Serial.println(F("[-] Successfully set nRF power to -6 dBm"));
      break;
    case '4':
      NRF_PA_LEVEL = RF24_PA_HIGH;
      Serial.println(F("[+] Successfully set nRF power to 0 dBm"));
      break;
  }

  switch(pipe_eeprom) {
    case '1':
      Serial.println(F("[+] Successfully set pipe address to 0xE056D446D0LL"));
      pipes[0] = 0xE056D446D0LL;
      break;
    case '2':
      pipes[0] = 0xE056D446D0LL;
      Serial.println(F("[+] Successfully set pipe address to 0xE056D446D0LL"));
      break;
    case '3':
      pipes[0] = 0xE056D446D0LL;
      Serial.println(F("[+] Successfully set pipe address to 0xE056D446D0LL"));
      break;
    case '4':
      pipes[0] = 0xE056D446D0LL;
      Serial.println(F("[+] Successfully set pipe address to 0xE056D446D0LL"));
      break;
  }

  switch(ical_eeprom) {
  }

  NRF_CHANNEL = nrf_channel_eeprom;
  Serial.print(F("[+] Successfully set nrf channel to "));
  Serial.println(NRF_CHANNEL);

  VOLTAGE = 1; //voltage_eeprom;
  Serial.print(F("[+] Successfully set mean voltage to "));
  Serial.print(VOLTAGE);
  Serial.println(F(" V"));

  NUMBER_SAMPLES_I = 50; //number_samples_i_eeprom;
  Serial.print(F("[+] Successfully set number of samples to " ));
  Serial.print(NUMBER_SAMPLES_I);
  Serial.println(F(" sample/s"));

}

///////////////////////////////
//   Loop part of the code   //
///////////////////////////////

void loop(void)
{
  if(inMenu != 0) {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    menu();
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
  } 
  else {
    //
    // Read Current
    //
    digitalWrite(GREEN_LED_PIN, HIGH);
    nrf.power = (int) (emon1.calcIrms(1480) * 240.0);
    digitalWrite(GREEN_LED_PIN, LOW);
    
    // Fix voltage
    nrf.voltage = 0; // 240;
    
    nrf.battery = (int) emon1.readVcc();
    
    //
    // Data sender
    //

    // First, stop listening so we can talk.
    radio.stopListening();

    if( DEBUG )
    {
      Serial.print("|");
      Serial.print(nrf.power);
      Serial.print("\t");
      Serial.print("|");
      Serial.print("\t");
      Serial.print(nrf.voltage);
      Serial.print("\t");
      Serial.print("|");
      Serial.print("\t");
      Serial.print(nrf.battery);
      Serial.print("\t");
      Serial.println("|");
    }
    
    radio.write(&nrf, sizeof(PayloadTX));

    // Now, continue listening now the goal is to get some data back as ACK
    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > TIMEOUT )
        timeout = true;

    // Describe the results
    if ( timeout && DEBUG )
    {
      Serial.println(F("[!] Failed to send packet : response timed out ..."));
    }
    
    // Stop listening
    radio.stopListening();
  }
  
  //
  // Shut down the system
  //

  // 100ms for the MCU to settle 
  delay(100);

  // Power down the radio.  Note that the radio will get powered back up
  // on the next write() call.
  // TODO : Fix this !
  //radio.powerDown();

  // Sleep the MCU.  The watchdog timer will awaken in a short while, and
  // continue execution here.
  do_sleep();
  
  // 100ms for the MCU to settle 
  delay(100);
  
}

/////////////////////////////
//   MENU CONFIGURATION    //
/////////////////////////////

void menu() {

  String result = "";
  int resultLength = 11;
  int resultat;

  if(inMenu == 1) {
    unsigned long time = millis();
    while((!Serial.available()) && (millis() - time < 10000)) { }
    input = Serial.read();
    if(input == 'Y' || input == 'y') {
      Serial.println();
      Serial.println(F("\t [1] Change nRF speed"));
      Serial.println(F("\t [2] Change nRF PA level"));
      Serial.println(F("\t [3] Change nRF channel"));
      Serial.println(F("\t [4] Change mean voltage (0 to enable measurements)"));
      Serial.println(F("\t [5] Change number of samples for intensity measurement"));
      Serial.println(F("\t [6] Change calibration factor"));
      Serial.println(F("\t [7] Change nRF's pipe address"));
      Serial.println(F("\t [9] Exit menu"));
      Serial.println(F("[*] When setting set, confirm by writing 'y'"));
      Serial.println(F("[*] these settings will be effective on next reboot"));
      Serial.println(F("[+] What is your choice ? (1...9) "));
      inMenu = 2;
      }
    else {
      inMenu = -1;
    }
  }

  if(inMenu == -1) {
    Serial.println(F("==================="));
    Serial.println(F("Starting measuring."));
    Serial.println();
    Serial.println(F("|P\t|\tV\t|\tBattery |"));
    inMenu = 0;
  }

  if(inMenu == 2) {
    while(!Serial.available()) { }
    input = Serial.read();
    Serial.println(input);
    char CharSwitch[resultLength];
    // TODO : Refactor this because resultLength is not used in the test cases
    switch(input) {
      case '1':
        Serial.println(F("[*] nRF speed in Kbps (1 for 250, 2 for 1024 or 3 for 2048) : "));
        while(!Serial.available()) {}
        input = Serial.read();
        EEPROM.write(NRF_SPEED_EEPROM, (int)input);
        Serial.println(F("[*] Press 'y' to confirm"));
        inMenu = 1;
        break;

      case '2':
        Serial.println(F("[!] WARNING : changing PA level can considerably increase power consumption !"));
        Serial.println(F("[*] nRF PA level (1 for -18, 2 for -12, 3 for -6 or 4 for 0 dBm) : "));
        while(!Serial.available()) {}
        input = Serial.read();
        EEPROM.write(NRF_PA_LEVEL_EEPROM, (int)input);
        Serial.println(F("[*] Press 'y' to confirm"));
        inMenu = 1;
        break;

      case '3':
        Serial.println(F("[*] nRF channel : "));
        while(!Serial.available()) {}
        result = "";
        while(Serial.available()) { result.concat((char)Serial.read()); }
        resultLength = result.length()+1;
        result.toCharArray(CharSwitch,resultLength);
        resultat = atoi(CharSwitch)*2;
        EEPROM.write(NRF_CHANNEL_EEPROM, resultat);
        Serial.println(F("[*] Press 'y' to confirm"));
        inMenu = 1;
       break;

      case '4':
        Serial.println(F("[*] mean voltage : "));
        while(!Serial.available()) {}
        result = "";
        while(Serial.available()) { result.concat((char)Serial.read()); }
        resultLength = result.length()+1;
        result.toCharArray(CharSwitch,resultLength);
        resultat = atoi(CharSwitch)*2;
        EEPROM.write(VOLTAGE_EEPROM, resultat);
        Serial.println(F("[*] Press 'y' to confirm"));
        inMenu = 1;
        break;

      case '5':
        Serial.println(F("[*] number of samples : "));
        while(!Serial.available()) {}
        result = "";
        while(Serial.available()) { result.concat((char)Serial.read()); }
        resultLength = result.length()+1;
        result.toCharArray(CharSwitch,resultLength);
        resultat = atoi(CharSwitch)*2;
        EEPROM.write(NUMBER_SAMPLES_I_EEPROM, resultat);
        Serial.println(F("[*] Press 'y' to confirm"));
        inMenu = 1;
        break;

      case '6':
        inMenu = 1;
        Serial.println(F("[*] Press 'y' to confirm"));
        break;

      case '7':
        inMenu = 1;
        Serial.println(F("[*] Press 'y' to confirm"));
        break;

      case '9':
        inMenu = -1;
        break;

    }
  }
}

//////////////////////////
//   Sleep functions    //
//////////////////////////

// 0=16ms, 1=32ms,2=64ms,3=125ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec

void setup_watchdog(uint8_t prescalar)
{
  prescalar = min(9,prescalar);
  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);

  MCUSR &= ~_BV(WDRF);
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);
}

ISR(WDT_vect)
{
}

void do_sleep(void)
{
  //cbi(ADCSRA, ADEN);                   // Disable ADC

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out

  //sbi(ADCSRA, ADEN);                   // Enable ADC
}



