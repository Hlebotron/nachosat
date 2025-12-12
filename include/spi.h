int read_radio( struct RadioData* data );
int read_sd( struct SDData* data );
int write_sd( struct SDData* data );

void SpiTask( void* params );
