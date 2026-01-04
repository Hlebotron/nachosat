#include <HardwareSerial.h>

extern QueueHandle_t spi_drq = xQueueCreate( 32, sizeof(SpiDataRequest) );
extern QueueHandle_t i2c_drq = xQueueCreate( 32, sizeof(I2CDataRequest) );
extern QueueHandle_t chute_drq = xQueueCreate( 32, sizeof(ChuteDataRequest) );
extern QueueHandle_t uart_out_queue = xQueueCreate( 32, sizeof(RadioData) );

#include "main.h"
#include "i2c.cpp"
#include "spi.cpp"
#include "chute.cpp"
#include "data_clock.cpp"
#include "uart.cpp"

void vApplicationIdleHook()
{
    for( ;; )
    {

    }
}

void setup()
{
    Serial.begin( 115200 );

    int *spi_param, *i2c_param, *chute_param, *clock_param, *uart_param;
    xTaskCreatePinnedToCore( SpiTask, "SpiTask", 10000, spi_param, 1, NULL, 0 );
    // xTaskCreatePinnedToCore( I2CTask, "I2CTask", 10000, i2c_param, 1, NULL, 0 );
    // xTaskCreatePinnedToCore( UartTask, "UartTask", 10000, uart_param, 1, NULL, 0 );
    // xTaskCreatePinnedToCore( ParachuteTask, "ParachuteTask", 10000, chute_param, 2, NULL, 1 );
    // xTaskCreatePinnedToCore( DataClockTask, "DataClockTask", 10000, clock_param, 1, NULL, 1 );

    
}
void loop() {}
