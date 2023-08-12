#include <Arduino.h>
//#include "SoftwareSerial.h"
#include "vt_lora"

using namespace vt;

//SoftwareSerial ss(8, 9);

lora_e22 lora_tx(Serial3, 2, 4);
lora_e22 lora_rx(Serial1, 3, 5);
//lora_e22 lora_rx(ss, 3, 5);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);

    // Begin configuration (sleep) mode.
    lora_e22 *loras[] = {&lora_tx, &lora_rx};
    for (auto &lora: loras) {
        Serial.println("Beginning configuration mode...");
        lora->config();
        Serial.println("Entered configuration mode!");
        lora->set_param(0xffff,
                        0,
                        9600,
                        LoRaParity::PARITY_8N1,
                        2400,
                        63,
                        240,
                        true);

        lora->query_param();

        Serial.println("Done config!");
    }

    lora_tx.begin(9600);
    lora_rx.begin(9600);
}

void loop() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.println("Sent!");

    // Test tx
    lora_tx.hw_serial().println("Message");

    // Test rx
    while (lora_rx.hw_serial().available()) Serial.write(lora_rx.hw_serial().read());

    delay(1000);
}
