#include "uart.h"
#include "main.h"

extern QueueHandle_t uart_out_queue;

int read_radio( struct RadioData& data )
{

}

#ifdef GROUND

void UartTask( void* params )
{
    // attachInterrupt( 35,  );
    HardwareSerial radio_uart( 1 );

    radio_uart.begin( UART_BAUD, SERIAL_8N1, COMMS_UART_RX, COMMS_UART_TX );
    *(BaseType_t*) params = 3;
    String str;
    BaseType_t str_len = 0;
    for( ;; )
    {
	while( radio_uart.available() > 0 )
	    str += (char) radio_uart.read();
	
	while( radio_uart.availableForWrite() > 0 )
	{
	    
	}

    }
}

#else

void UartTask( void* params )
{
    
}

#endif
