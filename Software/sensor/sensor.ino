 ///////////////////
//   Includes    //
///////////////////

#include <EEPROM.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"
#include "RF24.h"

///////////////////////
//   CONFIGURATION   //
///////////////////////

#define DEBUG 1

// input is for reading serial inputs, inMenu for menu navigation 
// SERIAL_BAUDRATE is for setting the baudrate for serial communication
char input;
int inMenu = 0;
int SERIAL_BAUDRATE;
#define BAUDRATE_EEPROM 7

// Time (ms) to allow the filters to settle before sending data
// const int FILTER_SETTLE_TIME = 5000;

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

// Calibration factor for the intensity
double ICAL = 111.1;
#define ICAL_EEPROM 5

// Radio pipe addresses for the 2 nodes to communicate.
uint64_t pipe = 0xE056D446D0LL;
#define PIPE_EEPROM 6


///////////////////////
//   Declarations    //
///////////////////////

#define TIMEOUT 250 // timeout in ms

// Struct to send RF data
typedef struct {
    int intensity;
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

const short sleep_cycles_per_transmission = 1; // Number of seconds to sleep between 2 events
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//////////////////////////
//   Setup operation    //
//////////////////////////

void setup(void)
{
  Serial.begin(57600);
  Serial.println("/!\\ STARTING CitizenOS");
  
  // Read the config stored on EEPROM
  int nrf_speed_eeprom = EEPROM.read(NRF_SPEED_EEPROM);
  int nrf_pa_level_eeprom = EEPROM.read(NRF_PA_LEVEL_EEPROM);
  int nrf_channel_eeprom = EEPROM.read(NRF_CHANNEL_EEPROM);
  int voltage_eeprom = EEPROM.read(VOLTAGE_EEPROM);
  int number_samples_i_eeprom = EEPROM.read(NUMBER_SAMPLES_I_EEPROM);
  int ical_eeprom = EEPROM.read(ICAL_EEPROM);
  int pipe_eeprom = EEPROM.read(PIPE_EEPROM);
  int baudrate_eeprom = EEPROM.read(BAUDRATE_EEPROM);

  Serial.println("//////////////////////////////");
  Serial.println("//    CitizenWatt sensor    //");
  Serial.println("//    citizenwatt.paris     //");
  Serial.println("////////////////////////////// \n");
  Serial.println("Configuration:");
  Serial.println("==============================");
  
  // Load the config
  load(nrf_speed_eeprom, nrf_pa_level_eeprom, nrf_channel_eeprom, 
       voltage_eeprom, number_samples_i_eeprom, ical_eeprom, 
       pipe_eeprom, baudrate_eeprom);
       
  Serial.println();
  Serial.println("[+] Go to the config menu ? y/[n]");
  inMenu = 1;
  
  //
  // Prepare sleep parameters
  //

  setup_watchdog(wdt_2s); // /!\ 2s sleeping

  //
  // Setup and configure rf radio
  //

  // Initialize nRF
  radio.begin();
  radio.setChannel(NRF_CHANNEL);  

  // Max number of retries and max delay between them
  // radio.setRetries(15, 15);

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
  radio.openWritingPipe(0xE056D446D0LL);

  //
  // Start listening
  //

  radio.startListening();  
}

void load(int nrf_speed_eeprom, int nrf_pa_level_eeprom, int nrf_channel_eeprom, int voltage_eeprom, 
         int number_samples_i_eeprom, int ical_eeprom, int pipe_eeprom, int baudrate_eeprom)
{
  switch(nrf_speed_eeprom) {
    case '1':
      NRF_SPEED = RF24_250KBPS;
      Serial.println("[+] Successfully set nRF speed to 250 kbps");
      break;
    case '2':
      NRF_SPEED = RF24_1MBPS;
      Serial.println("[+] Successfully set nRF speed to 1 mbps");
      break;
    case '3':
      Serial.println("[+] Successfully set nRF speed to 2 mbps");
      NRF_SPEED = RF24_2MBPS;
      break;
  }
  switch(nrf_pa_level_eeprom) {
    case '1':
      Serial.println("[+] Successfully set nRF power to -18 dBm");
      NRF_PA_LEVEL = RF24_PA_MIN;
      break;
    case '2':
      NRF_PA_LEVEL = RF24_PA_LOW;
      Serial.println("[+] Successfully set nRF power to -12 dBm");
      break;
    case '3':
      //NRF_PA_LEVEL = RF24_PA_MED;
      Serial.println("[-] Successfully set nRF power to -6 dBm");
      break;
    case '4':
      NRF_PA_LEVEL = RF24_PA_HIGH;
      Serial.println("[+] Successfully set nRF power to 0 dBm");
      break;
  }
  switch(nrf_channel_eeprom) {
    case '1':
      Serial.println("[+] Successfully set nrf channel to ?");
      NRF_CHANNEL = 76;
      break;
    case '2':
      NRF_CHANNEL = 76;
      Serial.println("[+] Successfully set nrf channel to ?");
      break;
    case '3':
      NRF_CHANNEL = 76;
      Serial.println("[+] Successfully set nrf channel to ?");
      break;
    case '4':
      NRF_CHANNEL = 76;
      Serial.println("[+] Successfully set nrf channel to ?");
      break;
  }        
  switch(voltage_eeprom) {
    case '1':
      Serial.println("[+] Successfully set mean voltage to 210V");
      VOLTAGE = 210;
      break;
    case '2':
      VOLTAGE = 220;
      Serial.println("[+] Successfully set mean voltage to 220V");
      break;
    case '3':
      VOLTAGE = 230;
      Serial.println("[+] Successfully set mean voltage to 230V");
      break;
    case '4':
      VOLTAGE = 240;
      Serial.println("[+] Successfully set mean voltage to 240V");
      break;
  }       
  switch(number_samples_i_eeprom) {
    case '1':
      Serial.println("[+] Successfully set number of samples to 1470 sample/s");
      NUMBER_SAMPLES_I = 210;
      break;
    case '2':
      NUMBER_SAMPLES_I = 220;
      Serial.println("[+] Successfully set number of samples to 1480 sample/s");
      break;
    case '3':
      NUMBER_SAMPLES_I = 230;
      Serial.println("[+] Successfully set number of samples to 1490 sample/s");
      break;
    /*case '4':
      NUMBER_SAMPLES_I = 240;
      Serial.println("[+] Successfully set number of samples to 240V");
      break;*/
  }
      
  /*switch(pipe_eeprom) {  
    case '1':
      Serial.println("[+] Successfully set pipe address to 0xE056D446D0LL");
      pipe = 0xE056D446D0LL;
      break;
    case '2':
      pipe = 0xE056D446D0LL;
      Serial.println("[+] Successfully set pipe address to 0xE056D446D0LL");
      break;
    case '3':
      pipe = 0xE056D446D0LL;
      Serial.println("[+] Successfully set pipe address to 0xE056D446D0LL");
      break;
    case '4':
      pipe = 0xE056D446D0LL;
      Serial.println("[+] Successfully set pipe address to 0xE056D446D0LL");
      break;
  }*/
  
  /*switch(ical_eeprom) {  //TODO
  }
  
  switch(baudrate_eeprom) {
    case '1':
      Serial.println("[+] Successfully set baudrate to 9600 baud");
      SERIAL_BAUDRATE = 9600;
      break;
    case '2':
      SERIAL_BAUDRATE = 19200;
      Serial.println("[+] Successfully set baudrate to 19200 baud");
      break;
    case '3':
      SERIAL_BAUDRATE = 57600;
      Serial.println("[+] Successfully set baudrate to 57600 baud");
      break;
    case '4':
      SERIAL_BAUDRATE = 115200;
      Serial.println("[+] Successfully set baudrate to 115200 baud");
      break;
  }   */
  
}

///////////////////////////////
//   Loop part of the code   //
///////////////////////////////

void loop(void)
{
  if(inMenu != 0) {
    menu();
  }
  else {
    //
    // Data sender
    //
    
    // First, stop listening so we can talk.
    //radio.stopListening();
    
    Serial.print("|");
    //Serial.print(nrf.intensity);
    Serial.print("\t");
    Serial.print("|");
    Serial.print("\t");
    //Serial.print(nrf.voltage);
    Serial.print("\t");
    Serial.print("|");
    Serial.print("\t");
    //Serial.print(nrf.battery);
    Serial.print("\t");
    Serial.print("|");
    Serial.println();
  
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
      Serial.println("[!] Failed to send packet : response timed out ...");
    }
  
    //
    // Shut down the system
    //
  
    // Experiment with some delay here to see if it has an effect
    delay(500);
  
    // Power down the radio.  Note that the radio will get powered back up
    // on the next write() call.
    radio.powerDown();
  /*
    // Sleep the MCU.  The watchdog timer will awaken in a short while, and
    // continue execution here.
    while( sleep_cycles_remaining )
      do_sleep();
  
    sleep_cycles_remaining = sleep_cycles_per_transmission;
    delay(1000);
  */
  }
}

/////////////////////////////
//   MENU CONFIGURATION    //
/////////////////////////////

void menu() {
  if(inMenu == 1) {
    while(!Serial.available()) { }
    input = Serial.read();
    if(input == 'Y' || input == 'y') {
      Serial.println();
      Serial.println("\t [1] Change nRF speed");
      Serial.println("\t [2] Change nRF PA level");
      Serial.println("\t [3] Change serial baudrate");
      Serial.println("\t [4] Change nRF channel");
      Serial.println("\t [5] Change mean voltage (0 to enable measurements)");
      Serial.println("\t [6] Change number of samples for intensity measurement");
      Serial.println("\t [7] Change calibration factor");
      Serial.println("\t [8] Change nRF's pipe address");
      Serial.println("\t [9] Exit menu");
      Serial.println("[*] When setting set, confirm by writing 'y'");
      Serial.println("[*] these settings will be effective on next reboot");
      Serial.println("[+] What is your choice ? (1...9) ");
      inMenu = 2;
      }
    else {
      inMenu = -1;
    }
  }
  
  if(inMenu == -1) { 
    Serial.println("===============");
    Serial.println("Starting measuring.");
    Serial.println();
    Serial.println("|I\t|\tV\t|\tBattery |");
    Serial.print("|");
    //Serial.println(inMenu);
    //Serial.print(nrf.intensity);
    Serial.print("\t");
    Serial.print("|");
    Serial.print("\t");
    //Serial.print(nrf.voltage);
    Serial.print("\t");
    Serial.print("|");
    Serial.print("\t");
    //Serial.print(nrf.battery);
    Serial.print("\t");
    Serial.print("|");
    Serial.println();
    inMenu = 0;
  }
  
  switch(inMenu) {
    case 2:
      while(!Serial.available()) { }
      input = Serial.read();
      Serial.println(input);
      switch(input) {
      case '1':
        inMenu = 11;
        break;
      case '2':
        inMenu = 12;
        break;
      case '3':
        inMenu = 13;
        break;
      case '4':
        inMenu = 14;
        break;
      case '5':
        inMenu = 15;
        break;
      case '6':
        inMenu = 16;
        break;
      case '7':
        inMenu = 17;
        break;
      case '8':
        inMenu = 18;
        break;
      case '9':
        inMenu = -1;
        break;
      }
      break;
      
    case 11:
      Serial.println("[*] nRF speed in Kbps (1 for 250, 2 for 1024 or 3 for 2048) : ");
      while(!Serial.available()) {}
      input = Serial.read();
      EEPROM.write(NRF_SPEED_EEPROM, (int)input);
      Serial.write("[*] Press 'y' to confirm");
      inMenu = 1;
      break;
      
    case 12:
      Serial.println("[!] WARNING : changing PA level can considerably increase power consumption !");
      Serial.println("[*] nRF PA level (1 for -18, 2 for -12, 3 for -6 or 4 for 0 dBm) : ");
      while(!Serial.available()) {}
      input = Serial.read();
      EEPROM.write(NRF_PA_LEVEL_EEPROM, (int)input);
      Serial.write("[*] Press 'y' to confirm");
      inMenu = 1;
      break;
      
    case 13: 
      Serial.println("[*] serial baudrate (1 for 9600, 2 for 19200 3 for 57600 or 4 for 115200): ");
      while(!Serial.available()) {}
      input = Serial.read();
      EEPROM.write(BAUDRATE_EEPROM, (int)input);
      Serial.write("[*] Press 'y' to confirm");
      inMenu = 1;
      break;
      
    case 14:
      Serial.println("[*] nRF channel : ");
      while(!Serial.available()) {}
      input = Serial.read();
      EEPROM.write(NRF_CHANNEL_EEPROM, (int)input);
      Serial.write("[*] Press 'y' to confirm");
      inMenu = 1;
     break; 
     
    case 15:
      Serial.println("[*] mean voltage : ");
      while(!Serial.available()) {}
      input = Serial.read();
      EEPROM.write(VOLTAGE_EEPROM, (int)input);
      Serial.write("[*] Press 'y' to confirm");
      inMenu = 1;
      break; 
      
    case 16:
      Serial.println("[*] number of samples : ");
      while(!Serial.available()) {}
      input = Serial.read();
      EEPROM.write(NUMBER_SAMPLES_I_EEPROM, (int)input);
      Serial.write("[*] Press 'y' to confirm");
      inMenu = 1;
      break; 
      
    case 17:
      inMenu = 1;
      Serial.write("[*] Press 'y' to confirm");
      break;
      
    case 18:
      inMenu = 1;
      Serial.write("[*] Press 'y' to confirm");
      break;
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
  --sleep_cycles_remaining;
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

/////////////////////////////////
//   Data reading functions    //
/////////////////////////////////

// Returns the battery voltage
//
// params : source is either VCC (battery) or EXTERNAL (external voltage) or a value for the MUX register
// See http://openenergymonitor.blogspot.it/2012/08/low-level-adc-control-admux.html
//

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


// Returns the measured intensity (root mean squared)
//
// params : samples_number is the number of samples used for averaging
// See http://openenergymonitor.org/emon/buildingblocks/digital-filters-for-offset-removal
//
// TODO: Use integer arithmetic
//
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
        filtered_I = 0.996 * (last_filtered_I + sample_I - last_sample_I);

        sum_I += filtered_I * filtered_I;
    }

    return ICAL * sqrt(sum_I / samples_number);
}

