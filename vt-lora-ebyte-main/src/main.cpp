#include <Arduino.h>
#include "vt_lora"

using namespace vt;

lora_e32 lora(Serial1, 22, 23);

void setup() {
    // Begin configuration (sleep) mode.
    Serial.begin(9600);
    lora.config();
    lora.set_param(0,               // Address
                   9600,          // Operational baud rate
                   LoRaParity::PARITY_8N1,           // Serial parity
                   9600,         // Air data rate
                   35,             // Frequency channel
                   LoRaTxPower::TX_MAX,               // Tx Power
                   false,        // Enable Forward Error Correction?
                   true);     // Save configuration for next boot?

    
    // Begin operational mode.
    lora.query_param();
    lora.begin(9600);

}

void loop() {}
