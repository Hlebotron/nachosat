#include <HardwareSerial.h>

#include "i2c_addresses.cpp"
#include "config.h"

void setup()
{
    Wire.begin( I2C_SDA, I2C_SCL, I2C_FREQ);
    Serial.begin( DEBUG_BAUD );
    delay( 1000 );
    // while (!Serial);

    Serial.println( "\nI2C Scanner" );
}

void loop() {
    i2c_addresses();
    delay( 1000 );
}
