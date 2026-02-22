#include "uart.h"

extern QueueHandle_t uart_out_drq;
extern SemaphoreHandle_t uart_in_sem;
static QueueSetHandle_t uart_set = xQueueCreateSet( UART_OUT_LEN + 1 );

void uart_irq()
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR( uart_in_sem, &higher_priority_task_woken );
    portYIELD_FROM_ISR( higher_priority_task_woken );
}

#ifdef GROUND

void UartTask( void* params )
{
    attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_irq, FALLING );
    HardwareSerial radio_uart( 1 );

    radio_uart.begin( UART_RADIO_BAUD, SERIAL_8N1, UART_RADIO_RX, UART_RADIO_TX );
    *(BaseType_t*) params = 3;
    String str;
    BaseType_t str_len = 0;
    for( ;; )
    {
	//Forward all messages
	while( radio_uart.available() > 0 )
	    Serial.print( radio_uart.read() );
	Serial.println();

	while( radio_uart.availableForWrite() >= sizeof("General Kenobi") )
	{
	    radio_uart.println( "General Kenobi" );
	    delay( 1000 );
	}

    }
}

#else

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
    attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_irq, FALLING );

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
