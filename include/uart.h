#ifdef GROUND

#else

#endif

int read_radio( struct RadioData& data );
int write_radio( const struct RadioData& data );

void UartTask( void* params );
