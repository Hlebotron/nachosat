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
//     Wire.beginTransmission(QMC5883L_ADDR);
//     Wire.write(reg);
//     Wire.write(val);
//     return Wire.endTransmission() == 0;
// }

// static bool i2c_read(uint8_t reg, uint8_t *buf, uint8_t len) {
//     Wire.beginTransmission(QMC5883L_ADDR);
//     Wire.write(reg);
//     if (Wire.endTransmission(false) != 0) return false;
//     Wire.requestFrom(QMC5883L_ADDR, len);
//     for (uint8_t i = 0; i < len; i++) {
//         if (!Wire.available()) return false;
//         buf[i] = Wire.read();
//     }
//     return true;
// }

// bool hmc5883l_init() {
//     // Config A: 8-sample avg, 15 Hz output, normal measurement
//     if (!i2c_write(QMC5883L_REG_CONFIG_A, 0x70)) return false;

//     // Config B: Gain = 1.3 Ga (default)
//     if (!i2c_write(QMC5883L_REG_CONFIG_B, 0x20)) return false;

//     // Continuous measurement mode
//     if (!i2c_write(QMC5883L_REG_MODE, QMC5883L_MODE_CONTINUOUS)) return false;

//     data_ready = false;
//     return true;
// }

// bool hmc5883l_read(hmc5883l_data_t *out) {
//     if (!data_ready) return false;

//     uint8_t buf[6];
//     if (!i2c_read(QMC5883L_REG_DATA_X_MSB, buf, 6)) return false;

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
int bmi160_write( uint8_t reg, uint8_t data )
{
    Wire.beginTransmission( I2C_BMI_ADDR );
    if( !Wire.write( reg ) ) return -1;
    if( !Wire.write( data ) ) return -1;
    return Wire.endTransmission();
}

int bmi160_read8( uint8_t reg, uint8_t& out )
{
    Wire.beginTransmission( I2C_BMI_ADDR );
    if( Wire.write( reg ) == 0 ) return -1;
    int res = Wire.endTransmission( false );
    if( res != 0 ) return res;
    if( Wire.requestFrom( I2C_BMI_ADDR, 1 ) == 0 ) return -2;
    out = Wire.read();
    return 0;
}

int bmi160_read16( uint8_t reg, int16_t& out )
{
    uint8_t lsb, msb;
    int res = bmi160_read8( reg, lsb );
    if( res != 0 ) return res;
    res = bmi160_read8( reg + 1, msb );
    if( res != 0 ) return res;
    out = ( int16_t )( (msb << 8) | lsb );
    return 0;
}

/* =========================
   BMI INIT
   /   ========================= */

int bmi160_init()
{
    uint8_t id;
    int res = write_reg2(
	I2C_BMI_ADDR,
	BMI_CHIP_ID,
	id,
	true );
    if( res != 0 ) return res;
    if ( id != 0xD1 ) return -3;

    // Accelerometer normal mode
    res = write_reg2(
	I2C_BMI_ADDR,
	BMI_CMD,
	0x11,
	true );
    if( res != 0 ) return res;
    delay(50);

    // Gyroscope normal mode
    res = write_reg2(
	I2C_BMI_ADDR,
	BMI_CMD,
	0x15
	true );
    if( res != 0 ) return res;
    delay(50);

    // ACC: 100 Hz, normal BW
    res = write_reg2(
	I2C_BMI_ADDR,
	BMI_ACC_CONF,
	0x28,
	true );
    if( res != 0 ) return res;

    // ACC: ±2g
    res = write_reg2(
	I2C_BMI_ADDR,
	BMI_ACC_RANGE,
	0x03,
	true );
    if( res != 0 ) return res;

    // GYRO: 100 Hz
    res = write_reg2(
	I2C_BMI_ADDR,
	BMI_GYR_CONF,
	0x28,
	true );
    if( res != 0 ) return res;

    // GYRO: ±2000 dps
    res = write_reg2(
	I2C_BMI_ADDR,
	BMI_GYR_RANGE,
	0x00,
	true );
    if( res != 0 ) return res;

    return true;
}

int read_bmi160( AccelData& accel )
{
    int16_t ax, ay, az, gx, gy, gz;
    int res = bmi160_read16( BMI_ACC_X_L, ax );
    if( res != 0 ) return res;
    res = bmi160_read16( BMI_ACC_X_L + 2, ay );
    if( res != 0 ) return res;
    res = bmi160_read16( BMI_ACC_X_L + 4, az );
    if( res != 0 ) return res;
    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    
    res = bmi160_read16( BMI_GYR_X_L, gx );
    if( res != 0 ) return res;
    res = bmi160_read16( BMI_GYR_X_L + 2, gy );
    if( res != 0 ) return res;
    res = bmi160_read16( BMI_GYR_X_L + 4, gz );
    if( res != 0 ) return res;
    float gx_dps = gx / 16.4f;
    float gy_dps = gy / 16.4f;
    float gz_dps = gz / 16.4f;

    float roll  = atan2( ay_g, az_g ) * 57.2958;
    float pitch = atan2( -ax_g, sqrt(ay_g * ay_g + az_g * az_g) ) * 57.2958;

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

    return 0;
}


int write_reg1( uint8_t dev_addr, uint8_t val, bool endstop )
{
    int res;
    Wire.beginTransmission( dev_addr );
    if( Wire.write(val) == 0 ) return -1;
    res = Wire.endTransmission( endstop );
    if( res != 0 ) return res;
    return 0;
}

int write_reg2( uint8_t dev_addr, uint8_t addr, uint8_t val, bool endstop )
{
    int res;
    Wire.beginTransmission( dev_addr );
    if( Wire.write(addr) == 0 ) return -1;
    if( Wire.write(val) == 0 ) return -2;
    res = Wire.endTransmission( endstop );
    if( res != 0 ) return res;
    return 0;
}

int magneto_init()
{
    int res;
    //According to documentation:
    //8-average, 15 Hz default, normal measurement
    res = write_reg2(
	I2C_QMC_ADDR,
	QMC_FREQ_REG_ADDR,
	QMC_FREQ_REG_VALUE,
	true );
    if( res != 0 ) return res;

    res = write_reg2(
	I2C_QMC_ADDR,
	QMC_GAIN_REG_ADDR,
	QMC_GAIN_REG_VALUE,
	true );
    if( res != 0 ) return res;


#ifndef MAGNETO_SINGLE
    //Continuous-measurement mode
    res = write_reg2(
	I2C_QMC_ADDR,
	QMC_MODE_REG_ADDR,
	QMC_MODE_REG_VALUE,
	true );
    if( res != 0 ) return res;

    //Options:
    //1. Wait 6ms
    //2. Monitor status register (poll if I understand correctly)
    //3. Use DRDY hardware interrupt pin
    //Eventually it will be option 3, but for now it's option 1
    delay( 6 MS );
#endif
    
    return 0;
}

int read_magneto( MagnetoData& magneto )
{
    int err;
#ifdef MAGNETO_SINGLE
    write_reg2(
	I2C_QMC_ADDR,
	QMC_MODE_REG_ADDR,
	QMC_MODE_REG_VALUE,
	true );
    write_reg2(
	I2C_QMC_ADDR,
	QMC_MODE_REG_ADDR,
	QMC_MODE_REG_VALUE,
	true );

    //Switch to option 3
    delay( 6 MS );

    write_reg1(
	I2C_QMC_ADDR,
	0x02,
	false );
    Wire.requestBytes( I2C_QMC_ADDR, 6 );

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
		magneto.x = ( (Wire.read() << 8) | val ) * QMC_SCALE_AVG / QMC_SCALE_X
		break;
	    case 2:
		magneto.z = ( (Wire.read() << 8) | val ) * QMC_SCALE_AVG / QMC_SCALE_Y;
		break;
	    case 3:
		magneto.y = ( Wire.read() << 8 ) | val;
		break;
	    }
	}
    }
    
#else
    write_reg1(
	I2C_QMC_ADDR,
	0x06,
	false );
    Wire.requestFrom( I2C_QMC_ADDR, 6 );

    //Read values
    

    write_reg1(
	I2C_QMC_ADDR,
	0x03,
	true );
#endif
    return 0;
}

void I2CTask( void* params )
{

    Wire.begin( I2C_SDA, I2C_SCL, I2C_FREQ );

    bool bmi_up = bmi160_init();
    if( !bmi_up )
    {
	Serial.print( "Could not initialize BMI160" );
    }

    // res = magneto_init();
    // if( res != 0 )
    // {
    // 	Serial.println( "Could not initialize magnetometer, code: " );
    // 	Serial.println( res );	
    // }

    QMC5883P qmc;
    bool qmc_up = qmc.begin();
    if( !qmc_up )
	Serial.println( "Could not initialize QMC" );
    qmc.setHardIronOffsets( QMC_OFFSET_X, QMC_OFFSET_Y );

    float roll, pitch;
    AccelData accel;
    MagnetoData magneto;
    int status;
    float xyz[3];
    float heading;
    for( ;; )
    {
	// if( bmi_up )
	// {
	//     read_bmi160( accel );
	
	//     Serial.printf( "ortho_x: %f\n", accel.ortho_x );
	//     Serial.printf( "ortho_y: %f\n", accel.ortho_y );
	//     Serial.printf( "ortho_z: %f\n", accel.ortho_z );
	//     Serial.printf( "gyro_x: %f\n", accel.gyro_x );
	//     Serial.printf( "gyro_y: %f\n", accel.gyro_y );
	//     Serial.printf( "gyro_z: %f\n", accel.gyro_z );
	//     Serial.printf( "roll: %f\n", accel.roll );
	//     Serial.printf( "pitch: %f\n\n", accel.pitch );
	// }
	// else
	//     Serial.println( "BMI160 is not up; not reading" );
	
	// status = read_magneto( magneto );
	// if( status != 0 )
	// {
	//     Serial.println( "Could not read magnetometer" );
	//     continue;
	// }
	// Serial.printf( "Magneto x: %i\n", magneto.x );
	// Serial.printf( "Magneto y: %i\n", magneto.y );
	// Serial.printf( "Magneto z: %i\n", magneto.z );

	if( qmc_up )
	{
	    if( qmc.readXYZ(xyz) )
	    {
		heading = fmod( qmc.getHeadingDeg(QMC_DECL_ANGLE) + QMC_ANGLE_OFFSET, 360 );
		Serial.printf( "X:%.2f  Y:%.2f  Z:%.2f µT  |  Heading: %3.0f°\n",
			       xyz[0], xyz[1], xyz[2], heading );
	    }
	}
	else
	    Serial.println( "QMC is not up; not reading" );
	
	delay( 500 MS );
    }
}
