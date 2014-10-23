 ///////////////////
//   Includes    //
///////////////////

#include <EEPROM.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <nRF24L01.h>
#include <RF24.h>      // https://github.com/stanleyseow/RF24
#include "printf.h"
#include <EmonLib.h>   // https://github.com/openenergymonitor/EmonLib
#include <AESLib.h>    // https://github.com/DavyLandman/AESLib

///////////////////////
//   CONFIGURATION   //
///////////////////////

#define DEBUG 1
#define SERIAL_BAUDRATE 57600

// PreAmplifier level for the nRF
// Lower this to reduce power consumption. This will reduce range.
rf24_pa_dbm_e NRF_PA_LEVEL = RF24_PA_LOW;

// Radio pipe addresses for the 2 nodes to communicate.
uint64_t pipes[2] = { 0xE056D446D0LL, 0xF0F0F0F0D2LL };

// AES
uint8_t key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

// Calibration factor for the intensity
double ICAL = 90.9;

// =============================================================

// Speed for the nrf module
// RF24_250KBPS / RF24_1MBPS / RF24_2MBPS
// Reduce it to improve reliability
rf24_datarate_e NRF_SPEED = RF24_1MBPS;

// Channel for the nrf module
// 76 is default safe channel in RF24
int NRF_CHANNEL = 0x4c;

// Set this to 0 to enable voltage measurement.
// Else, set this to your mean voltage.
double VOLTAGE = 240.0;

// Number of samples over which the mean must be done for the current measurement
int NUMBER_SAMPLES_I = 1480;

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

///////////////////////
//   Declarations    //
///////////////////////

#define TIMEOUT 250 // timeout in ms

// Struct to send RF data
typedef struct {
    int power;
    int voltage;
    int battery;
    unsigned long timer;
    long padding4;
    int padding2;
} PayloadTX;

// Next measurement to be sent
PayloadTX nrf = {0, 0, 0, 0, 0, 0};

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
  Serial.println(F("//////////////////////////////"));
  Serial.println(F("//    CitizenWatt sensor    //"));
  Serial.println(F("//    citizenwatt.paris     //"));
  Serial.println(F("////////////////////////////// \n"));

  if( DEBUG )
    printf_begin();

  //
  // Prepare sleep parameters
  //

  setup_watchdog(wdt_8s); // /!\ 8s sleeping

  //
  // Setup and configure rf radio
  //

  // Initialize nRF
  radio.begin();
  radio.setChannel(NRF_CHANNEL);

  // Max number of retries and max delay between them
  radio.setRetries(15, 15);

  // Reduce payload size to improve reliability
  radio.setPayloadSize(16);

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
  
  // Init EnergyMonitor, with calibration factor for R1 = 22 Ohms
  emon1.current(0, ICAL);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
}

///////////////////////////////
//   Loop part of the code   //
///////////////////////////////

void loop(void)
{
  //
  // Read Current
  //
  digitalWrite(GREEN_LED_PIN, HIGH);
  nrf.power = (int) (emon1.calcIrms(NUMBER_SAMPLES_I) * VOLTAGE);
  digitalWrite(GREEN_LED_PIN, LOW);
  
  // Set voltage
  nrf.voltage = VOLTAGE;
  
  // Read Vcc
  nrf.battery = (int) emon1.readVcc();
  
  // Adding random for AES
  nrf.timer = millis();
  
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
    
    Serial.print(F("Clear :"));      
    PrintHex8((uint8_t*)&nrf, sizeof(PayloadTX));
    Serial.println();
    Serial.flush();
  }
  
  // AES ciphering
  aes128_enc_single(key, &nrf);
  
  if( DEBUG )
  {
    Serial.print(F("Cipher:"));      
    PrintHex8((uint8_t*)&nrf, sizeof(PayloadTX));
    Serial.println();
    Serial.flush();
  }
    
  //
  // Data sender
  //

  // First, stop listening so we can talk.
  radio.stopListening();
       
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
    Serial.println(F("[!] Failed to send packet : response timed out ..."));
  
  // Stop listening
  radio.stopListening();
    
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
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out
}

// Function for debug
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
  Serial.print("0x"); 
  for (int i=0; i<length; i++) { 
    if (data[i]<0x10) {Serial.print("0");} 
    Serial.print(data[i],HEX); 
    Serial.print(" "); 
  }
}

