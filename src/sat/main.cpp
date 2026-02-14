#include <HardwareSerial.h>
#include <qmc5883p.h>

#include "config.h"
#include "definitions.h"

QueueHandle_t spi_drq = xQueueCreate( SPI_DRQ_LEN, sizeof(Peripheral) );
QueueHandle_t i2c_drq = xQueueCreate( I2C_DRQ_LEN, sizeof(Peripheral) );
QueueHandle_t uart_out_drq = xQueueCreate( UART_OUT_LEN, sizeof(RadioResponse) );
SemaphoreHandle_t uart_in_sem = xSemaphoreCreateBinary();

/* Corresponds to:
   BMP280
   Accelerometer
   Magnetometer
*/
TimerHandle_t timers[ NUM_SOURCES ];
// QueueHandle_t chute_drq = xQueueCreate( CHUTE_DRQ_LEN, sizeof(enum ChuteDataSource) );


#include "i2c.cpp"
#include "spi.cpp"
#include "chute.cpp"
// #include "data_clock.cpp"
#include "uart.cpp"


// void IdleHook()
// {
//     for( ;; )
//     {

//     }
// }
void TimerCallback( TimerHandle_t timer )
{
    Peripheral sensor;
    
    switch( (uint32_t) pvTimerGetTimerID(timer) )
    {
    case 0:
	sensor = PERI_BMP;
	xQueueSendToBack( spi_drq, &sensor, 0 );
	break;
    case 1:
	sensor = PERI_ACCEL;
	xQueueSendToBack( i2c_drq, &sensor, 0 );
	break;
    case 2:
	sensor = PERI_MAGNETO;
	xQueueSendToBack( i2c_drq, &sensor, 0 );
	break;
    }
}

void setup()
{
    Serial.begin( 115200 );

    for( int i = 0; i < NUM_SOURCES; i++ )
	timers[ i ] = xTimerCreate( "timer", DEFAULT_DATA_REQUEST_INTERVAL, pdTRUE, (void*) i, TimerCallback );

    int *spi_param, *i2c_param, *chute_param, *clock_param, *uart_param;
    xTaskCreatePinnedToCore( SpiTask, "SpiTask", 10000, spi_param, 1, NULL, 0 );
    xTaskCreatePinnedToCore( I2CTask, "I2CTask", 10000, i2c_param, 1, NULL, 0 );
    // xTaskCreatePinnedToCore( UartTask, "UartTask", 10000, uart_param, 1, NULL, 0 );
    // xTaskCreatePinnedToCore( ParachuteTask, "ParachuteTask", 10000, chute_param, 2, NULL, 1 );
    for( int i = 0; i < NUM_SOURCES; i++ )
	xTimerStart( timers[i], 100 );
}
void loop() {}
