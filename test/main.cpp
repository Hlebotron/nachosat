#include <HardwareSerial.h>

#include "i2c_addresses.cpp"
#include "config.h"

void setup()
{
    Wire.begin();
    Serial.begin( DEBUG_BAUD );
    delay( 1000 );
    // while (!Serial);

    Serial.println( "\nI2C Scanner" );
}

void loop() {
    i2c_addresses();
    delay( 1000 );
}
