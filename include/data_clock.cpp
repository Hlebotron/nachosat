#include "data_clock.h"

extern QueueHandle_t spi_drq;
extern QueueHandle_t i2c_drq;
extern QueueHandle_t chute_drq;

void DataClockTask( void* params )
{
    TickType_t pxPreviousWakeTime;
    enum SpiDataSource data;
    
    for( ;; )
    {
	
	xQueueSendToBack( spi_drq, &SPI_BMP, 10 MS );
	vTaskDelayUntil( &pxPreviousWakeTime, 50 MS );
    }
}
