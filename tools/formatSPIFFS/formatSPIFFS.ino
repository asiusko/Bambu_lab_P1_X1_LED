#include <SPIFFS.h>

 // format SPIFFS
void setup() {
    delay(1000);
    Serial.begin(115200);
    if (!SPIFFS.begin(true)) { // `true` will format SPIFFS if mounting fails
        Serial.println("Failed to mount or format file system");
        return;
    }
    Serial.println("SPIFFS mounted successfully");
}

void loop() {
    // nope
}

