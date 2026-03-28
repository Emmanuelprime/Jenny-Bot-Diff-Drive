#define PI_RX_PIN 23
#define PI_TX_PIN 19
#define PI_BAUD 115200

void setup() {
    Serial.begin(115200);
    Serial2.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);

    Serial.println("ESP32 Ready");
}

void loop() {
    // Read from Raspberry Pi
    if (Serial2.available()) {
        String msg = Serial2.readStringUntil('\n');
        Serial.print("Received: ");
        Serial.println(msg);
    }

    delay(10);
}