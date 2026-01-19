#include <Wire.h>



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
