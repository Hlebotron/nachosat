#include <HardwareSerial.h>
// #include <cbuf.cpp>

#include "config.h"
#include "definitions.h"
#define GROUND

// QueueHandle_t queue_from_uart = xQueueCreate( QUEUE_LEN, sizeof(RadioResponse) );
// QueueHandle_t queue_from_serial = xQueueCreate( QUEUE_LEN, sizeof(RadioRequest) );

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

void SerialTask( void* params )
{
    // //Create the queue set
    // static QueueSetHandle_t queue_set = xQueueCreateSet( QUEUE_LEN + 1 ); //Length of data_response_queue + serial_sem
    // xQueueAddToSet( serial_sem, queue_set );
    // xQueueAddToSet( queue_from_uart, queue_set );
    
    Serial.begin( UART_SERIAL_BAUD );

    *( (BaseType_t*) params ) = 1; //Set status of this task

    char radio_rx_buf[ STREAM_BUF_LEN ];
    char radio_tx_buf[ STREAM_BUF_LEN ];

    BaseType_t prev = *( (BaseType_t*) params );
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

    size_t
	read_available = 0,
	write_available = 0,
	write_available_tmp = 0,
	read_count = 0,
	write_count = 0;
    for( ;; )
    {
	if( xStreamBufferReceive(stream, radio_rx_buf, sizeof(radio_rx_buf), 0) > 0 ) // Receiving data from the radio
	{
	    
	}
	
	read_available = Serial.available();
	if( read_available > 0 ) // Receiving data from the laptop
	{
	    read_count = Serial.readBytes( radio_tx_buf, read_available );
	    if( read_count != read_available )
		Serial.printf( "Warning: Read %d bytes, expected %d\n", read_count, read_available );
	    read_available = 0;
	}

	// The upload code should be received 3 times to switch to upload mode (to upload code to the ESP32)

	// Why not just merge the UartTask and the SerialTask if the ground station is just a proxy?
	// Why does the ground station require software at all? Why not just use a USB-Serial converter? Perhaps for the sake of flexibility?
	write_available_tmp = radio_uart.availableForWrite();
	write_available = (write_available_tmp < write_count) ? write_available_tmp : write_count;
	if( write_available > 0 ) // Sending data to the radio
	{
	    size_t written = radio_uart.write( radio_tx_buf, write_available ); // Actually write
	    if( written != write_available ) 
		Serial.printf( "Warning: Wrote %d bytes, expected write of %d bytes\n", written, write_available );
	    // What should be done to the buffer after it has been written from? Move all the elements to the beginning? Does UART track buffer positions?
	}
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
