#include "uart.h"

extern QueueHandle_t uart_out_queue;
extern SemaphoreHandle_t uart_in_sem;
static QueueSetHandle_t uart_set = xQueueCreateSet();

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
	    str += (char) radio_uart.read();

	while( radio_uart.availableForWrite() > 0 )
	{
	    
	}
    }
}

#else

void UartTask( void* params )
{
    HardwareSerial radio_uart( 1 );
    radio_uart.begin( UART_RADIO_BAUD, SERIAL_8N1, UART_RADIO_RX, UART_RADIO_TX );

    xQueueAddToSet( uart_in_sem, uart_set );
    xQueueAddToSet( uart_out_queue, uart_set );
    
    attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_irq, FALLING );

    *(BaseType_t*) params = 3;
    String str;
    BaseType_t str_len = 0;
    QueueSetMemberHandle_t member;
    for( ;; )
    {
	member = xQueueSelectFromSet( uart_set, portMAX_DELAY );
	if( member == uart_in_sem )
	{
	    
	}
	else if( member == uart_out_queue )
	{
		
	}
    }
}

#endif
