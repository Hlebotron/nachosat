#include <Wire.h>
#include <HardwareSerial.h>
#include "definitions.h"

void i2c_addresses()
{
    char error, address;
    int deviceCount = 0;

    Serial.println("Scanning...");

    char addrs[] = {
      I2C_BMI_ADDR
    };

    for (int i = 0; i < ( sizeof(addrs) / sizeof(char) ); i++) {
	address = addrs[i];
	Wire.beginTransmission(address);
	error = Wire.endTransmission();

	if (error == 0) {
	    Serial.print("I2C device found at address 0x");
	    if (address < 16)
		Serial.print("0");
	    Serial.print(address, HEX);
	    Serial.println(" !");
	    deviceCount++;
	}
	else if (error == 4) {
	    Serial.print("Unknown error at address 0x");
	    if (address < 16)
		Serial.print("0");
	    Serial.println(address, HEX);
	}
    }

    if (deviceCount == 0)
	Serial.println("No I2C devices found\n");
    else
	Serial.println("Scan complete\n");
}
