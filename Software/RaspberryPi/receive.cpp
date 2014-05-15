#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <time>
#include "./RF24.h"

const bool DEBUG = true;

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

const uint64_t pipe = 0xE056D446D0LL;

// Struct to receive RF data
typedef struct {
    int intensity;
    int voltage;
    int battery;
} PayloadTX;

RF24 radio(RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_24, BCM2835_SPI_SPEED_8MHZ);

void handlePayload(PayloadTX payload) {
    if(DEBUG) {
        time(&timer);

        std::cout << "[" << difftime(start, timer) << "]" << "Packet received.\n";
        std::cout << "Intensity : " << payload.intensity << "\t";
        std::cout << "Voltage : " << payload.voltage << "\t";
        std::cout << "Battery : " << payload.battery << "\t";
    }
}

int main() {
    PayloadTX payload = {0, 0, 0};
    time_t start = time(NULL);
    time_t timer;

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
    // Open reading pipe
    radio.openReadingPipe(pipe);

    radio.startListening();

    while(1) {
        if(radio.available()) {
            radio.read(&payload, sizeof(PayloadTX));

            handlePayload(payload);
        }
    }
}
