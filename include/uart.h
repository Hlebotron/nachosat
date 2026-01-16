#define UART_BAUD 	( 115200 )
#define COMMS_UART_RX 	( 16 )
#define COMMS_UART_TX 	( 17 )

#ifdef GROUND

#else

#endif

int read_radio( struct RadioData& data );
int write_radio( const struct RadioData& data );

void UartTask( void* params );
