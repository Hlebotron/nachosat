#include <HardwareSerial.h>
#include <Arduino.h>
// #include <>
// #include <cbuf.cpp>

#include "config.h"
#include "definitions.h"

#define GROUND
// #define configTASK_NOTIFICATION_ARRAY_ENTRIES 4

// QueueHandle_t queue_from_uart = xQueueCreate( QUEUE_LEN, sizeof(RadioResponse) );
// QueueHandle_t queue_from_serial = xQueueCreate( QUEUE_LEN, sizeof(RadioRequest) );

TaskHandle_t uart_handle;

SemaphoreHandle_t serial_sem = xSemaphoreCreateBinary();
SemaphoreHandle_t uart_sem = xSemaphoreCreateBinary();

// StreamBufferHandle_t stream = xStreamBufferCreate( STREAM_BUF_LEN, STREAM_BUF_TRIG );

#include "uart.cpp"

void IdleHook()
{
    esp_sleep_enable_uart_wakeup( 1 );
    for( ;; )
    {
	
	esp_light_sleep_start();
    }
}

void UartTask( void* params )
{
    attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_isr, FALLING );
    attachInterrupt( digitalPinToInterrupt(UART_SERIAL_RX), serial_isr, FALLING );
    HardwareSerial radio_uart( 1 );
    
    // //Create the queue set
    // static QueueSetHandle_t queue_set = xQueueCreateSet( QUEUE_LEN + 1 ); //Length of data_response_queue + serial_sem
    // xQueueAddToSet( serial_sem, queue_set );
    // xQueueAddToSet( queue_from_uart, queue_set );
    
    Serial.begin( UART_SERIAL_BAUD );
    radio_uart.begin( UART_RADIO_BAUD );

    // *( (BaseType_t*) params ) = 1; //Set status of this task

    char radio_rx_buf[ STREAM_BUF_LEN ];
    char radio_tx_buf[ STREAM_BUF_LEN ];

    Serial.println( "Starting" );
    for( ;; )
    {
	uint32_t bits = 0;
	xTaskNotifyWait( 0, ULONG_MAX, &bits, portMAX_DELAY );
	Serial.printf( "Radio: %u, Serial: %u\n",
	    (uint16_t)( bits & BIT_RADIO ),
	    (uint16_t)( (bits & BIT_SERIAL) >> 16 ) );
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
