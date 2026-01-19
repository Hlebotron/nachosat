#include <HardwareSerial.h>
#include <Wire.h>
#include "i2c.h"
#include <math.h>
// extern Semaph
// oreHandle_t received_sem = xSemaphoreCreateBinary();
extern QueueHandle_t i2c_drq;

// void received_isr()
// {
//     xSemaphoreGiveFromISR( received_queue, NULL );
// }

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

// #include "hmc5883l.h"


// static volatile bool data_ready = false;

// void IRAM_ATTR hmc5883l_drdy_isr() {
//     data_ready = true;
// }

// static bool i2c_write(uint8_t reg, uint8_t val) {
//     Wire.beginTransmission(HMC5883L_ADDR);
//     Wire.write(reg);
//     Wire.write(val);
//     return Wire.endTransmission() == 0;
// }

// static bool i2c_read(uint8_t reg, uint8_t *buf, uint8_t len) {
//     Wire.beginTransmission(HMC5883L_ADDR);
//     Wire.write(reg);
//     if (Wire.endTransmission(false) != 0) return false;
//     Wire.requestFrom(HMC5883L_ADDR, len);
//     for (uint8_t i = 0; i < len; i++) {
//         if (!Wire.available()) return false;
//         buf[i] = Wire.read();
//     }
//     return true;
// }

// bool hmc5883l_init() {
//     // Config A: 8-sample avg, 15 Hz output, normal measurement
//     if (!i2c_write(HMC5883L_REG_CONFIG_A, 0x70)) return false;

//     // Config B: Gain = 1.3 Ga (default)
//     if (!i2c_write(HMC5883L_REG_CONFIG_B, 0x20)) return false;

//     // Continuous measurement mode
//     if (!i2c_write(HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS)) return false;

//     data_ready = false;
//     return true;
// }

// bool hmc5883l_read(hmc5883l_data_t *out) {
//     if (!data_ready) return false;

//     uint8_t buf[6];
//     if (!i2c_read(HMC5883L_REG_DATA_X_MSB, buf, 6)) return false;

//     out->x = (int16_t)((buf[0] << 8) | buf[1]);
//     out->z = (int16_t)((buf[2] << 8) | buf[3]);
//     out->y = (int16_t)((buf[4] << 8) | buf[5]);

//     data_ready = false;
//     return true;
// }


// void I2CTask( void* params )
// {
//     // TickType_t prev_wake = xTaskGetTickCount();

//     Wire.begin();
//     QueueSetHandle_t queue_set = xQueueCreateSet( QUEUE_LEN + 1 );
//     xQueueAddToSet( received_queue, queue_set );
//     xQueueAddToSet( i2c_queue, queue_set );

//     static char c = '\0';
    
//     for( ;; )
//     {
// 	QueueSetMemberHandle_t member = xQueueSelectFromSet( queue_set, portMAX_DELAY );
// 	if( member == received_sem ) //Receiving from I2C bus
// 	{
// 	    while( Wire.available() )
// 	    {
		
// 	    }
		
// 	}
// 	else if( member == i2c_drq ) //Receiving data request
// 	{
	    
// 	}
//     }
// }
void bmi160_write( uint8_t reg, uint8_t data )
{
    Wire.beginTransmission( I2C_BMI_ADDR );
    Wire.write( reg );
    Wire.write( data );
    Wire.endTransmission();
}

uint8_t bmi160_read8( uint8_t reg )
{
    Wire.beginTransmission( I2C_BMI_ADDR );
    Wire.write( reg );
    Wire.endTransmission( false );
    Wire.requestFrom( I2C_BMI_ADDR, 1 );
    return Wire.read();
}

int16_t bmi160_read16(uint8_t reg)
{
    uint8_t lsb = bmi160_read8(reg);
    uint8_t msb = bmi160_read8(reg + 1);
    return (int16_t)((msb << 8) | lsb);
}

/* =========================
   BMI INIT
   /   ========================= */

bool bmi160_init()
{
    uint8_t id = bmi160_read8( BMI_CHIP_ID );
    if ( id != 0xD1 )
	return false;

    // Accelerometer normal mode
    bmi160_write(BMI_CMD, 0x11);
    delay(50);

    // Gyroscope normal mode
    bmi160_write(BMI_CMD, 0x15);
    delay(50);

    // ACC: 100 Hz, normal BW
    bmi160_write(BMI_ACC_CONF, 0x28);

    // ACC: ±2g
    bmi160_write(BMI_ACC_RANGE, 0x03);

    // GYRO: 100 Hz
    bmi160_write(BMI_GYR_CONF, 0x28);

    // GYRO: ±2000 dps
    bmi160_write(BMI_GYR_RANGE, 0x00);

    return true;
}

int read_bmi160( AccelData& accel )
{
    float ax_g = bmi160_read16(BMI_ACC_X_L) / 16384.0f;
    float ay_g = bmi160_read16(BMI_ACC_X_L + 2) / 16384.0f;
    float az_g = bmi160_read16(BMI_ACC_X_L + 4) / 16384.0f;

    float gx_dps = bmi160_read16(BMI_GYR_X_L) / 16.4f;
    float gy_dps = bmi160_read16(BMI_GYR_X_L + 2) / 16.4f;
    float gz_dps = bmi160_read16(BMI_GYR_X_L + 4) / 16.4f;

    float roll  = atan2(ay_g, az_g) * 57.2958;
    float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 57.2958;

    accel = {
	ax_g,
	ay_g,
	az_g,
	gx_dps,
	gy_dps,
	gz_dps,
	roll,
	pitch
    };

    // Serial.print("ACC [g]  ");
    // Serial.print(ax_g, 3); Serial.print(" ");
    // Serial.print(ay_g, 3); Serial.print(" ");
    // Serial.print(az_g, 3);

    // Serial.print(" | GYR [dps] ");
    // Serial.print(gx_dps, 1); Serial.print(" ");
    // Serial.print(gy_dps, 1); Serial.print(" ");
    // Serial.println(gz_dps, 1);




    // Serial.print("Roll: "); Serial.print(roll, 2); Serial.print(" Pitch: "); Serial.println(pitch, 2);

    // Serial.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    // ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, roll, pitch);
    return 0;
}

int write_reg1( uint8_t dev_addr, uint8_t val, bool endstop )
{
    int res;
    Wire.beginTransmission( dev_addr );
    Serial.println( "Begin" );
    if( Wire.write(val) != 0 ) return -1;
    Serial.println( "1" );
    res = Wire.endTransmission( endstop );
    if( res != 0 ) return res;
    Serial.println( "2" );
    return 0;
}

int write_reg2( uint8_t dev_addr, uint8_t addr, uint8_t val, bool endstop )
{
    int res;
    Wire.beginTransmission( dev_addr );
    Serial.println( "Begin" );
    if( Wire.write(addr) != 0 ) return -1;
    Serial.println( "1" );
    if( Wire.write(val) != 0 ) return -2;
    Serial.println( "2" );
    res = Wire.endTransmission( endstop );
    if( res != 0 ) return res;
    Serial.println( "3" );
    return 0;
}

int magneto_init()
{
    int res;
    //According to documentation:
    //8-average, 15 Hz default, normal measurement
    write_reg2(
	I2C_HMC_ADDR,
	HMC_FREQ_REG_ADDR,
	HMC_FREQ_REG_VALUE,
	true );

    write_reg2(
	I2C_HMC_ADDR,
	HMC_GAIN_REG_ADDR,
	HMC_GAIN_REG_VALUE,
	true );


#ifndef MAGNETO_SINGLE
    //Continuous-measurement mode
    write_reg2(
	I2C_HMC_ADDR,
	HMC_MODE_REG_ADDR,
	HMC_MODE_REG_VALUE,
	true
	);

    delay( 6 MS );
#endif

    //Options:
    //1. Wait 6ms
    //2. Monitor status register (poll if I understand correctly)
    //3. Use DRDY hardware interrupt pin
    //Eventually it will be option 3, but for now it's option 1

    
    return true;
}

int read_magneto( MagnetoData& magneto )
{
    int err;
#ifdef MAGNETO_SINGLE
    write_reg2(
	I2C_HMC_ADDR,
	HMC_MODE_REG_ADDR,
	HMC_MODE_REG_VALUE,
	true );
    write_reg2(
	I2C_HMC_ADDR,
	HMC_MODE_REG_ADDR,
	HMC_MODE_REG_VALUE,
	true
	);


    //Switch to option 3
    delay( 6 MS );

    write_reg1(
	I2C_HMC_ADDR,
	0x02,
	false
	);

    Wire.requestBytes( I2C_HMC_ADDR, 6 );

    //Read values
    {
	uint8_t val = 0;
	for( int i = 1; i < 7; i++ )
	{
	    if( i % 2 == 1 )
	    {
		val = Wire.read();
		continue;
	    }

	    switch( i / 2 )
	    {
	    case 1:
		magneto.x = ( Wire.read() << 8 ) | val;
		break;
	    case 2:
		magneto.z = ( Wire.read() << 8 ) | val;
		break;
	    case 3:
		magneto.y = ( Wire.read() << 8 ) | val;
		break;
	    }
	}
    }
    
#else
    write_reg1(
	I2C_HMC_ADDR,
	0x06,
	false
	);
    Wire.requestFrom( I2C_HMC_ADDR, 6 );

    //Read values
    

    write_reg1(
	I2C_HMC_ADDR,
	0x03,
	true
	);
#endif
    return 0;
}

void I2CTask( void* params )
{

    
    Wire.begin( I2C_SDA, I2C_SCL, I2C_FREQ );

    if( !bmi160_init() )
    {
	Serial.println( "Could not initialize BMI160" );
    }


    if( !magneto_init() )
    {
	Serial.println( "Could not initialize magnetometer" );
    }

    float roll, pitch;
    AccelData accel;
    MagnetoData magneto;
    int status;
    for( ;; )
    {
	// read_bmi160( accel );
	
	// Serial.print( "ortho_x: " ); Serial.println( accel.ortho_x );
	// Serial.print( "ortho_y: " ); Serial.println( accel.ortho_y );
	// Serial.print( "ortho_z: " ); Serial.println( accel.ortho_z );
	// Serial.print( "gyro_x: " ); Serial.println( accel.gyro_x );
	// Serial.print( "gyro_y: " ); Serial.println( accel.gyro_y );
	// Serial.print( "gyro_z: " ); Serial.println( accel.gyro_z );
	// Serial.print( "roll: " ); Serial.println( accel.roll );
	// Serial.print( "pitch: " ); Serial.println( accel.pitch );
	// Serial.println();
	status = read_magneto( magneto );
	if( status != 0 )
	{
	    Serial.println( "Could not read magnetometer" );
	    continue;
	}
	Serial.print( "Magneto x: " ); Serial.println( magneto.x );
	Serial.print( "Magneto y: " ); Serial.println( magneto.y );
	Serial.print( "Magneto z: " ); Serial.println( magneto.z );
	
	delay( 500 MS );
    }
}
