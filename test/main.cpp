#include <Wire.h>
#include <HardwareSerial.h>
#include "i2c.h"

void setup()
{
    Wire.begin();
}
void loop()
{
    Serial.print( "Temperature: " );
    Serial.println( bmp_read_temp() );
    Serial.print( "Pressure: " );
    Serial.println( bmp_read_press() );
}
