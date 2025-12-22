#define MOSI 	17
#define MISO 	19
#define SCK 	16
#define BMP_SS 	18
#define SD_SS	20

int read_bmp( struct BMPData* data );
/* int read_radio( struct RadioData* data ); */
int read_sd( struct SDData* data );
int write_sd( struct SDData* data );

void SpiTask( void* params );
