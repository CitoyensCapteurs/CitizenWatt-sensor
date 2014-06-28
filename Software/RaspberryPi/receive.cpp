#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <mysql.h>
#include "../RF24.h"

const bool DEBUG = true;

// Speed for the nrf module
// RF24_250KBPS / RF24_1MBPS / RF24_2MBPS
// Reduce it to improve reliability
const rf24_datarate_e NRF_SPEED = RF24_1MBPS;

// PreAmplifier level for the nRF
// Lower this to reduce power consumption. This will reduce range.
const rf24_pa_dbm_e NRF_PA_LEVEL = RF24_PA_LOW;

// Channel for the nrf module
// 76 is default safe channel in RF24
const int NRF_CHANNEL = 0x4c;

const uint64_t addr = 0xF0F0F0F0E1LL;

// Struct to receive RF data
typedef struct {
    int power;
} PayloadTX;

//RF24 radio(RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_24, BCM2835_SPI_SPEED_8MHZ);
RF24 radio("/dev/spidev0.0",8000000 , 25);

int main() {
    PayloadTX payload = {0};
    time_t start = time(NULL);
MYSQL mysql;
mysql_init(&mysql);
mysql_options(&mysql, MYSQL_READ_DEFAULT_GROUP,"option");

if(!mysql_real_connect(&mysql, "localhost", "USER", "PASSWORD", "citizenwatt", 0, NULL, 0)) {
printf("Erreur lors de la connexion à la BDD");
exit(1);
}

    // Initialize nRF
    radio.begin();
    // Max number of retries and max delay between them
    radio.setRetries(15, 15);
    radio.setChannel(NRF_CHANNEL);
    // Reduce payload size to improve reliability
    //radio.setPayloadSize(8);
    // Set the datarate
    //radio.setDataRate(NRF_SPEED);
    // Use the largest CRC
    //radio.setCRCLength(RF24_CRC_16);
    // Ensure auto ACK is enabled
    //radio.setAutoAck(1);
    // Use the best PA level
    radio.setPALevel(NRF_PA_LEVEL);
    // Open reading pipe
    radio.openReadingPipe(1, addr);

    radio.startListening();

    while(1) {
        if(radio.available()) {
            radio.read(&payload, sizeof(PayloadTX));

	    if(DEBUG) {
		//std::cout << "[" << difftime(start, timer) << "]" << "Packet received.\n";
		std::cout << "Power : " << payload.power << "\n";
	    }

	    char request[150] = "";
	    sprintf(request, "INSERT INTO measures(id, date, power) VALUES('', NOW(), %d)", payload.power);
	    mysql_query(&mysql, request);
        }
    }
}
