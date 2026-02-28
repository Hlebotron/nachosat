// #include "task.h"
#include "uart.h"

extern QueueHandle_t uart_out_drq;
extern TaskHandle_t uart_handle;
extern SemaphoreHandle_t uart_in_sem;
static QueueSetHandle_t uart_set = xQueueCreateSet( UART_OUT_LEN + 1 );

void uart_isr()
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    // xTaskNotifyFromISR( uart_handle, 1, eSetBits, &higher_priority_task_woken ); // Actually notify the task
    vTaskNotifyGiveFromISR( uart_handle, &higher_priority_task_woken ); // Actually notify the task
    // vTaskNotifyGiveIndexedFromISR( uart_handle, 2, &higher_priority_task_woken2 ); // Notify the task about a message from the radio
    portYIELD_FROM_ISR( higher_priority_task_woken );
}

void serial_isr()
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR( uart_handle, &higher_priority_task_woken ); // Actually notify the task
    // xTaskNotifyFromISR( uart_handle, 1, eSetBits, &higher_priority_task_woken ); // Actually notify the task
    portYIELD_FROM_ISR( higher_priority_task_woken );
}

int write_radio( HardwareSerial& radio_uart, const RadioResponse* resp )
{
    static size_t written = 0;
    if( resp == NULL )
	return 2;
    
    switch( resp->sensor )
    {
    case PERI_ACCEL:
	written = radio_uart.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(AccelData)) );
	break;
    case PERI_GYRO:
	written = radio_uart.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(GyroData)) );
	break;
    case PERI_GPS:
	written = radio_uart.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(GPSData)) );
	break;
    case PERI_MAGNETO:
	written = radio_uart.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(MagnetoData)) );
	break;
    case PERI_BMP:
	written = radio_uart.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(BMPData)) );
	break;
    default:
	Serial.printf( "Invalid sensor, code: %i\n", resp->sensor );
	return 1;
    }
    
    if( written == 0 )
	return 3;
    
    return 0;
}

int radio_init( HardwareSerial& radio_uart )
{
    radio_uart.begin( UART_RADIO_BAUD, SERIAL_8N1, UART_RADIO_RX, UART_RADIO_TX );
    attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_isr, FALLING );

    radio_uart.write( "radio rxstop\r\n" ); 			delay( 5e5 );
    radio_uart.write( "radio set freq" );
    radio_uart.print( RADIO_FREQ );
    radio_uart.write( "\r\n" );					delay( 5e5 );
    radio_uart.write( "radio set mod lora\r\n" ); 		delay( 5e5 );
    radio_uart.write( "radio set sf sf7\r\n" ); 		delay( 5e5 );
    radio_uart.write( "radio set bw 125\r\n" ); 		delay( 5e5 );
    radio_uart.write( "radio set pa off\r\n" ); 		delay( 5e5 );
    radio_uart.write( "radio set pwr 10\r\n" ); 		delay( 5e5 );
    radio_uart.write( "radio rxstop\r\n" ); 			delay( 5e5 );
    radio_uart.write( "radio rx 0\r\n" ); 			delay( 5e5 );
    return 0;
}

#ifdef GROUND

// void UartTask( void* params )
// {
//     attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_isr, FALLING );
//     HardwareSerial radio_uart( 1 );

//     radio_uart.begin( UART_RADIO_BAUD, SERIAL_8N1, UART_RADIO_RX, UART_RADIO_TX );
//     *( BaseType_t* ) params = 3;
//     String str;
//     BaseType_t str_len = 0;
//     for( ;; )
//     {
// 	//Forward all messages
// 	while( radio_uart.available() > 0 )
// 	    Serial.print( radio_uart.read() );
// 	Serial.println();

// 	while( radio_uart.availableForWrite() >= sizeof("General Kenobi") )
// 	{
// 	    radio_uart.println( "General Kenobi" );
// 	    delay( 1000 );
// 	}

//     }
// }
// void UartTask( void* params )
// {
//     attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_isr, FALLING );
//     attachInterrupt( digitalPinToInterrupt(UART_SERIAL_RX), serial_isr, FALLING );
//     HardwareSerial radio_uart( 1 );
    
//     // //Create the queue set
//     // static QueueSetHandle_t queue_set = xQueueCreateSet( QUEUE_LEN + 1 ); //Length of data_response_queue + serial_sem
//     // xQueueAddToSet( serial_sem, queue_set );
//     // xQueueAddToSet( queue_from_uart, queue_set );
    
//     Serial.begin( UART_SERIAL_BAUD );
//     radio_uart.begin( UART_RADIO_BAUD );

//     // *( (BaseType_t*) params ) = 1; //Set status of this task

//     char radio_rx_buf[ STREAM_BUF_LEN ];
//     char radio_tx_buf[ STREAM_BUF_LEN ];

//     size_t
// 	read_available = 0,
// 	write_available = 0,
// 	write_available_tmp = 0,
// 	read_count = 0,
// 	write_count = 0;
//     for( ;; )
//     {
// 	if( xStreamBufferReceive(stream, radio_rx_buf, sizeof(radio_rx_buf), 0) > 0 ) // Receiving data from the radio
// 	{
	    
// 	}
	
// 	read_available = Serial.available();
// 	if( read_available > 0 ) // Receiving data from the laptop
// 	{
// 	    read_count = Serial.readBytes( radio_tx_buf, read_available );
// 	    if( read_count != read_available )
// 		Serial.printf( "Warning: Read %d bytes, expected %d\n", read_count, read_available );
// 	    read_available = 0;
// 	}

// 	// The upload code should be received 3 times to switch to upload mode (to upload code to the ESP32)

// 	// Why not just merge the UartTask and the SerialTask if the ground station is just a proxy?
// 	// Why does the ground station require software at all? Why not just use a USB-Serial converter? Perhaps for the sake of flexibility?
// 	write_available_tmp = radio_uart.availableForWrite();
// 	write_available = (write_available_tmp < write_count) ? write_available_tmp : write_count;
// 	if( write_available > 0 ) // Sending data to the radio
// 	{
// 	    size_t written = radio_uart.write( radio_tx_buf, write_available ); // Actually write
// 	    if( written != write_available ) 
// 		Serial.printf( "Warning: Wrote %d bytes, expected write of %d bytes\n", written, write_available );
// 	    // What should be done to the buffer after it has been written from? Move all the elements to the beginning? Does UART track buffer positions?
// 	}
//     }
// }


#else



void UartTask( void* params )
{
    HardwareSerial radio_uart( 1 );
    radio_init( radio_uart );

    xQueueAddToSet( uart_in_sem, uart_set );
    xQueueAddToSet( uart_out_drq, uart_set );

    *( BaseType_t* ) params = 3;
    QueueSetMemberHandle_t member;
    RadioResponse resp;
    for( ;; )
    {
	member = xQueueSelectFromSet( uart_set, ( 1000 MS ) );
	if( member == uart_in_sem ) //If a message has arrived
	{
	    xSemaphoreTake( uart_in_sem, TICKS_TO_WAIT );
	    
	    while( radio_uart.available() )
	    {
		Serial.print( radio_uart.read() );
	    }
	}
	else if( member == uart_out_drq ) //If we want to send something
	{
	    BaseType_t res = xQueueReceive( uart_out_drq, (void*) &resp, TICKS_TO_WAIT );
	    write_radio( radio_uart, &resp );
	}
	else
	{
	    Serial.println( "Invalid queue, unable to determine name" );
	    Serial.println( "Sending bogus data ");
	    radio_uart.println( "Hello there" );
	    Serial.println( "Sent bogus data ");
	}
    }
}

#endif
