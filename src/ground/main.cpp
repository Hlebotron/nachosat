#include <HardwareSerial.h>

#define GROUND
#include "uart.cpp"

// QueueHandle_t queue_from_uart = xQueueCreate( QUEUE_LEN, sizeof(RadioResponse) );
// QueueHandle_t queue_from_serial = xQueueCreate( QUEUE_LEN, sizeof(RadioRequest) );

SemaphoreHandle_t serial_sem = xSemaphoreCreateBinary();
SemaphoreHandle_t uart_sem = xSemaphoreCreateBinary();

StreamBufferHandle_t stream = xStreamBufferCreate( STREAM_BUF_LEN, STREAM_BUF_TRIG );

void IdleHook()
{
    esp_sleep_enable_uart_wakeup( 1 );
    for( ;; )
    {
	
	esp_light_sleep_start();
    }
}

void SerialTask( void* params )
{
    // //Create the queue set
    // static QueueSetHandle_t queue_set = xQueueCreateSet( QUEUE_LEN + 1 ); //Length of data_response_queue + serial_sem
    // xQueueAddToSet( serial_sem, queue_set );
    // xQueueAddToSet( queue_from_uart, queue_set );
    
    Serial.begin( 115200 );

    *((BaseType_t*) params) = 1; //Set status of this task

    char radio_rx_buf[ STREAM_BUF_LEN ];
    char radio_tx_buf[ STREAM_BUF_LEN ];

    BaseType_t prev = *((BaseType_t*) params);
    Serial.print( "\nUART task starting" );
    while( *((BaseType_t*) params) != 3 )
    {
	if( *((BaseType_t*) params) != prev )
	{
	    Serial.print( "\nUART task created, setting up" );
	}
	Serial.print( "." );
	delay( 200 );
    }
    Serial.println( "\nUART task started" );
    for( ;; )
    {
	// xQueueReceive( queue_set, portMAX_DELAY );
	if( xStreamBufferReceive( stream,  ) > 0 )

	// Serial.print( "pog" );
    }
}




void setup()
{

    BaseType_t
	task_status = 0,
	creation_status = 0;
    
    do
	creation_status = xTaskCreate( SerialTask, "SerialTask", 10000, &task_status, 1, NULL );
    while( creation_status != pdPASS );
    while( task_status == 0 );
    // if( esp_register_freertos_idle_hook(myTestIdleHook) != ESP_OK )
	// ;
    
    do
	creation_status = xTaskCreate( UartTask, "UartTask", 10000, &task_status, 1, NULL );
    while( creation_status != pdPASS );
    task_status = 2;
    while( task_status == 0 );
    task_status = 3;
}

void loop() {}
