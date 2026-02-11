#include <SPI.h>
#include <Adafruit_BMP280.h>

// #include "spi.h"

extern QueueHandle_t spi_drq;
extern QueueHandle_t uart_out_drq;

int32_t read_bmp( Adafruit_Sensor* bmp_temp, Adafruit_Sensor* bmp_pressure, struct BMPData& data )
{
    if( bmp_temp == NULL )
	return 1;
    if( bmp_pressure == NULL )
	return 2;
    
    sensors_event_t temp_event, pressure_event;
    
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);


    data = { temp_event.temperature, pressure_event.pressure };
    return 0;
}

void SpiTask( void* params )
{
    /*
      I think H in HSPI implies Hardware and V in VSPI implies Virtual SPI
      In case of HSPI (probably not the case), the following pins MUST be used:
      14 (SCK)
      12 (MISO)
      13 (MOSI)
      15 (SS)
    */
    Adafruit_BMP280 bmp = Adafruit_BMP280( SPI_BMP_SS, SPI_MOSI, SPI_MISO, SPI_SCK );
    unsigned bmp_up = bmp.begin();
    if( !bmp_up )
    {
	Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
			 "try a different address!"));
	Serial.print( "Error code: " ); Serial.println( bmp_up );
	Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
	Serial.println("Expected SensorID for BMP280: 0x56-0x58\n");
    }
    Adafruit_Sensor* bmp_temp = bmp.getTemperatureSensor();
    Adafruit_Sensor* bmp_press = bmp.getPressureSensor();

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
		    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
		    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
		    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
		    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    

    BMPData data = { -500.0, -500.0 };

    enum Peripheral queue_val;
    struct RadioResponse resp;
    
    for( ;; )
    {
	xQueueReceive( spi_drq, &queue_val, portMAX_DELAY );
	//TODO: If BMP remains the only SPI device outside of the SD card reader, then remove the switch case and replace the queue it with a counting semaphore
	switch( queue_val ) {
	case PERI_BMP:
	    if( bmp_up )
	    {
	    int32_t read_status = read_bmp( bmp_temp, bmp_press, data );
	    switch ( read_status )
	    {
	    case 1:
		Serial.println( "BMP Read Error: bmp_temp is null" );
		continue;
	    case 2:
		Serial.println( "BMP Read Error: bmp_pressure is null" );	    
		continue;
	    }
	    
	    resp.sensor = PERI_BMP;
	    resp.data.bmp = { data.temp, data.pressure };
	    xQueueSendToBack( uart_out_drq, &resp, TICKS_TO_WAIT );
	    Serial.printf(  "\nTemp: %f\n", data.temp );
	    Serial.printf(  "Pressure: %f\n", data.pressure );
	    }
	    else
	    {
		bmp_up = bmp.begin();
		Serial.println( (bmp_up) ? "Successfully started BMP280" : "Failed to start BMP280" );
	    }
	    break;
	default:
	    Serial.printf( "Invalid sensor, value: %i\n", queue_val );
	}

    }
}
