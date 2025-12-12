#include <Wire.h>
#include <HardwareSerial.h>
#include "i2c.h"

void setup()
{
    Serial.print( "BMP_TEMP: " );
    Serial.println( BMP_TEMP );
    Wire.begin();
}
void loop()
{
    Serial.print( "Temperature: " );
    Serial.println( bmp_read_temp() );
    Serial.print( "Pressure: " );
    Serial.println( bmp_read_press() );
}
