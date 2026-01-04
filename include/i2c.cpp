#include <HardwareSerial.h>
#include <Wire.h>
#include "i2c.h"

// float bmp_read_press()
// {
//     Serial.println( "Reading pressure" );
//     Serial.println( "Begin transmission" );
//     Wire.beginTransmission( I2C_BMP_ADDR );
    
//     Serial.println( "Write" );
//     size_t written = Wire.write( BMP_PRESS );
//     if( written == 0 )
//     {
// 	Serial.print( "No data written to write buffer" );
// 	return -1.0;
//     }
    
//     Serial.println( "End transmission" );
//     uint8_t err = Wire.endTransmission();
//     if ( err != 0 )
//     {
// 	switch( err )
// 	{
// 	case I2C_ERR_NACK_ADDR_TRANSMIT:
// 	    Serial.println( "Error: Received NACK on transmit of address" );
// 	    break;
// 	case I2C_ERR_NACK_DATA_TRANSMIT:
// 	    Serial.println( "Error: Received NACK on transmit of data" );
// 	    break;
// 	case I2C_ERR_LINE_BUSY:
// 	    Serial.println( "Error: Line busy" );
// 	    break;
// 	}
// 	return ( err * (-1.0) );
//     }
    
//     Serial.println( "Begin transmission" );
//     Wire.beginTransmission( I2C_BMP_ADDR );
    
//     Serial.println( "Request" );
//     size_t read = Wire.requestFrom( I2C_BMP_ADDR, 2 );
//     if( read == 0 )
//     {
// 	Serial.print( "No data read from read buffer" );
// 	return -2.0;	
//     }

//     Serial.println( "End transmission" );
//     err = Wire.endTransmission();
//     if ( err != 0 )
//     {
// 	switch( err )
// 	{
// 	case I2C_ERR_NACK_ADDR_TRANSMIT:
// 	    Serial.println( "Error: Received NACK on transmit of address" );
// 	case I2C_ERR_NACK_DATA_TRANSMIT:
// 	    Serial.println( "Error: Received NACK on transmit of data" );
// 	case I2C_ERR_LINE_BUSY:
// 	    Serial.println( "Error: Line busy" );
// 	}
// 	return ( err * (-1.0) );
//     }

//     Serial.println( "Read 1" );
//     int tmp1 = Wire.read();
//     if( tmp1 == -1 )
//     {
// 	Serial.println( "Failed to read byte 1" );
//     }
    
//     Serial.println( "Read 2" );
//     int tmp2 = Wire.read();
//     if( tmp2 == -1 )
//     {
// 	Serial.println( "Failed to read byte 2" );
//     }
    



//     return ( ( tmp1 << 8 ) + tmp2 ) * BMP_MAX_RES_PRESS * ( float )( pow( 2, ( BMP_MAX_RES_BITS - BMP_RES_BITS ) ) );
// }

// float bmp_read_temp()
// {
//     //TODO: Error handling
//     Serial.println( "Reading temperature" );
//     Serial.println( "Begin transmission" );
//     Wire.beginTransmission( I2C_BMP_ADDR );
//     Serial.println( "Write" );
//     Wire.write( BMP_TEMP );
//     Serial.println( "End transmission" );
//     Wire.endTransmission();

//     Serial.println( "Begin transmission" );
//     Wire.beginTransmission( I2C_BMP_ADDR );
//     Serial.println( "Request" );
//     Wire.requestFrom( I2C_BMP_ADDR, 2 );
//     Serial.println( "Read 1" );
//     uint16_t tmp = Wire.read();
//     Serial.println( "Read 2" );
//     tmp |= ( Wire.read() << 8 );
//     Serial.println( "End transmission" );
//     Wire.endTransmission();

//     return tmp * BMP_MIN_RES_TEMP / ( float )( pow( 2, ( BMP_RES_BITS - BMP_MIN_RES_BITS ) ) );
// }

void I2CTask( void* params )
{
    TickType_t prev_wake = xTaskGetTickCount();
    for( ;; )
    {
	vTaskDelayUntil( &prev_wake, 500 MS );
    }
}
