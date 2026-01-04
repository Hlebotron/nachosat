#include <Wire.h>

#define BMP_TEMP		( 0xFA )
#define BMP_PRESS		( 0xF7 )
#define BMP_MIN_RES_BITS	( 16 )
#define BMP_MAX_RES_BITS	( 20 )
#define BMP_MIN_RES_TEMP	( 0.0050 )
#define BMP_MAX_RES_PRESS	( 0.16 )
#define BMP_RES_BITS		( 16 )

#define I2C_SCL			( 22 )
#define I2C_SDA			( 23 )
#define I2C_BMP_ADDR		( 0x76 )
#define I2C_GPS_ADDR		( NULL )
#define I2C_ACCEL_ADDR		( NULL )
#define I2C_ORIENT_ADDR		( NULL )

enum i2c_error
{
    I2C_OK = 0,
    I2C_ERR_NACK_ADDR_TRANSMIT = 2,
    I2C_ERR_NACK_DATA_TRANSMIT = 3,
    I2C_ERR_LINE_BUSY = 4
};

float bmp_read_temp();
float bmp_read_press();
int gps_read( struct GPSData& data );
int magnet_read( struct MagnetData& data );
int accel_read( struct AccelData& data );


void I2CTask( void* params );
