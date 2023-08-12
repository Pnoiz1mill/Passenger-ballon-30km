#include <Arduino.h>
#include "vt_lora"

using namespace vt;

lora_e32 lora(Serial1, 8, 9);

void setup() {
    // Begin configuration (sleep) mode.
    lora.config();
    lora.set_param(0,               // Address
                   115200,          // Operational baud rate
                   LoRaParity::PARITY_8N1,           // Serial parity
                   2400,         // Air data rate
                   0,             // Frequency channel
                   LoRaTxPower::TX_MAX,               // Tx Power
                   false,        // Enable Forward Error Correction?
                   true);     // Save configuration for next boot?

    // Begin operational mode.
    lora.begin(115200);
}

void loop() {}
