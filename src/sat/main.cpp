#include <HardwareSerial.h>
#include <qmc5883p.h>
#include "config.h"
#include "definitions.h"

#include "i2c.cpp"
#include "spi.cpp"
// #include "chute.cpp"
// #include "data_clock.cpp"
// #include "uart.cpp"

QueueHandle_t spi_drq = xQueueCreate( QUEUE_LEN, sizeof(enum SpiDataSource) );
// QueueHandle_t i2c_drq = xQueueCreate( QUEUE_LEN, sizeof(struct I2CDataSource) );
// QueueHandle_t chute_drq = xQueueCreate( QUEUE_LEN, sizeof(struct ChuteDataSource) );
// QueueHandle_t uart_out_queue = xQueueCreate( QUEUE_LEN, sizeof(struct RadioResponse) );
// QueueHandle_t uart_in_queue = xQueueCreate( QUEUE_LEN, sizeof(union RadioRequest) );

// void IdleHook()
// {
//     for( ;; )
//     {

//     }
// }

void setup()
{
    Serial.begin( 115200 );

    int *spi_param, *i2c_param, *chute_param, *clock_param, *uart_param;
    // xTaskCreatePinnedToCore( SpiTask, "SpiTask", 10000, spi_param, 1, NULL, 0 );
    xTaskCreatePinnedToCore( I2CTask, "I2CTask", 10000, i2c_param, 1, NULL, 0 );
    // xTaskCreatePinnedToCore( UartTask, "UartTask", 10000, uart_param, 1, NULL, 0 );
    // xTaskCreatePinnedToCore( ParachuteTask, "ParachuteTask", 10000, chute_param, 2, NULL, 1 );
    // xTaskCreatePinnedToCore( DataClockTask, "DataClockTask", 10000, clock_param, 1, NULL, 1 );

    
}
void loop() {}
