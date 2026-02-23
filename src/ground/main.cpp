#include <HardwareSerial.h>
// #include <cbuf.cpp>

#include "config.h"
#include "definitions.h"
#define GROUND

// QueueHandle_t queue_from_uart = xQueueCreate( QUEUE_LEN, sizeof(RadioResponse) );
// QueueHandle_t queue_from_serial = xQueueCreate( QUEUE_LEN, sizeof(RadioRequest) );

TaskHandle_t uart_handle;

SemaphoreHandle_t serial_sem = xSemaphoreCreateBinary();
SemaphoreHandle_t uart_sem = xSemaphoreCreateBinary();

StreamBufferHandle_t stream = xStreamBufferCreate( STREAM_BUF_LEN, STREAM_BUF_TRIG );

#include "uart.cpp"

void IdleHook()
{
    esp_sleep_enable_uart_wakeup( 1 );
    for( ;; )
    {
	
	esp_light_sleep_start();
    }
}


void setup()
{

    BaseType_t uart_status;
    
    // do
    // 	creation_status = xTaskCreate( SerialTask, "SerialTask", 10000, &task_status, 1, NULL );
    // while( creation_status != pdPASS );
    // while( task_status == 0 );
    // // if( esp_register_freertos_idle_hook(myTestIdleHook) != ESP_OK )
    // 	// ;
    
    // do
    xTaskCreate( UartTask, "UartTask", 10000, &uart_status, 1, &uart_handle );
    // while( creation_status != pdPASS );
    // task_status = 2;
    // while( task_status == 0 );
    // task_status = 3;
}

void loop() {}
