#include <HardwareSerial.h>
#include <Wire.h>
#include "i2c.h"
#include <math.h>
// extern SemaphoreHandle_t received_sem = xSemaphoreCreateBinary();
// extern QueueHandle_t i2c_drq;

static uint8_t sensor_fails[] = { 0, 0 }; //BMI160, QMC

int bmi160_read8( uint8_t reg, uint8_t& out )
{
    Wire.beginTransmission( I2C_BMI_ADDR );
    if( !Wire.write( reg ) ) return -1;
    int res = Wire.endTransmission( false );
    if( res != 0 ) return res;
    if( !Wire.requestFrom( I2C_BMI_ADDR, 1 ) ) return -2;
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

int write_reg1( uint8_t dev_addr, uint8_t val, bool endstop )
{
    int res;
    Wire.beginTransmission( dev_addr );
    if( !Wire.write(val) ) return -1;
    res = Wire.endTransmission( endstop );
    if( res != 0 ) return res;
    return 0;
}

int write_reg2( uint8_t dev_addr, uint8_t addr, uint8_t val, bool endstop )
{
    int res;
    Wire.beginTransmission( dev_addr );
    if( !Wire.write(addr) ) return -1;
    if( !Wire.write(val) ) return -2;
    res = Wire.endTransmission( endstop );
    if( res != 0 ) return res;
    return 0;
}

int bmi160_init()
{
    uint8_t id;
    int res = bmi160_read8( BMI_CHIP_ID, id );
    if( res != 0 ) return ( res < 0 ) ? (res - 3) : res;
    if ( id != 0xD1 ) return - 3;

    // Accelerometer normal mode
    res = write_reg2(
	I2C_BMI_ADDR,
	BMI_CMD,
	0x11,
	true );
    if( res != 0 ) return res;
    delay( 50 );

    // Gyroscope normal mode
    res = write_reg2(
	I2C_BMI_ADDR,
	BMI_CMD,
	0x15,
	true );
    if( res != 0 ) return res;
    delay( 50 );

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

    return 0;
}

int read_bmi160_accel( AccelData& accel )
{
    int16_t x, y, z;
    int res = bmi160_read16( BMI_ACC_X_L, x );
    if( res != 0 ) return res;
    res = bmi160_read16( BMI_ACC_X_L + 2, y );
    if( res != 0 ) return res;
    res = bmi160_read16( BMI_ACC_X_L + 4, z );
    if( res != 0 ) return res;
    float x_g = x / 16384.0f;
    float y_g = y / 16384.0f;
    float z_g = z / 16384.0f;

    float roll  = atan2( y_g, z_g ) * 57.2958;
    float pitch = atan2( -x_g, sqrt(y_g * y_g + z_g * z_g) ) * 57.2958;

    accel = {
	x_g,
	y_g,
	z_g,
	roll,
	pitch
    };

    return 0;
}


int read_bmi160_gyro( GyroData& gyro )
{
    int16_t x, y, z;
    int res = bmi160_read16( BMI_GYR_X_L, x );
    if( res != 0 ) return res;
    res = bmi160_read16( BMI_GYR_X_L + 2, y );
    if( res != 0 ) return res;
    res = bmi160_read16( BMI_GYR_X_L + 4, z );
    if( res != 0 ) return res;
    float x_dps = x / 16.4f;
    float y_dps = y / 16.4f;
    float z_dps = z / 16.4f;

    gyro = {
	x_dps,
	y_dps,
	z_dps
    };
    
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
    int res;
#ifdef MAGNETO_SINGLE
    res = write_reg2( I2C_QMC_ADDR, QMC_MODE_REG_ADDR,	QMC_MODE_REG_VALUE, true );
    if( res != 0 ) return res;
    // write_reg2(	I2C_QMC_ADDR, QMC_MODE_REG_ADDR,	QMC_MODE_REG_VALUE, true );

    //Switch to option 3
    delay( 6 MS );

    res = write_reg1( I2C_QMC_ADDR, 0x02, false );
    if( res != 0 ) return res;
    res = Wire.requestFrom( I2C_QMC_ADDR, 6 );
    if( res <= 0 ) return -1;

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
		magneto.x = ( (Wire.read() << 8) | val ) * QMC_SCALE_AVG / QMC_SCALE_X;
		break;
	    case 2:
		magneto.z = ( (Wire.read() << 8) | val ) * QMC_SCALE_AVG / QMC_SCALE_Y;
		break;
	    case 3:
		magneto.y = ( Wire.read() << 8 ) | val;
		break;
	    }
	    magneto.head = fmod( qmc.getHeadingDeg(QMC_DECL_ANGLE) + QMC_ANGLE_OFFSET, 360.0 );
	}
    }
    
#else
    res = write_reg1( I2C_QMC_ADDR, 0x06, false );
    if( res != 0 ) return res;
    res = Wire.requestFrom( I2C_QMC_ADDR, 6 );
    if( res != 0 ) return -1;

    //Read values
    

    res = write_reg1( I2C_QMC_ADDR, 0x03, true );
    if( res != 0 ) return res;
#endif
    return 0;
}

void I2CTask( void* params )
{
    // Wire.begin( I2C_SDA, I2C_SCL, I2C_FREQ );
    Wire.begin();
    delay( 200 );

    int bmi_err = bmi160_init();
    if( bmi_err == 0 )
    {
	Serial.println( "Successfully started BMI160" );
    }
    else
    {
	Serial.printf( "Could not initialize BMI160, code: %d\n", bmi_err );
    }

    // res = magneto_init();
    // if( res != 0 )
    // {
    // 	Serial.println( "Could not initialize QMC, code: " );
    // 	Serial.println( res );	
    // }

    QMC5883P qmc;
    bool qmc_up = qmc.begin();
    if( qmc_up )
    {
	Serial.println( "Successfully started QMC" );
    }
    else
    {
	Serial.println( "Could not initialize QMC" );
    }

    qmc.setHardIronOffsets( QMC_OFFSET_X, QMC_OFFSET_Y );

    float roll, pitch;
    AccelData accel;
    GyroData gyro;
    MagnetoData magneto;
    int status;
    float xyz[3];
    float heading;
    Peripheral queue_val;
    RadioResponse resp;
    
    for( ;; )
    {
	xQueueReceive( i2c_drq,  &queue_val, portMAX_DELAY );

	switch( queue_val )
	{
	case PERI_MAGNETO:
	    if( qmc_up && sensor_fails[0] < SENSOR_FAIL_THRESHOLD )
	    {
		if( qmc.readXYZ(xyz) )
		{

		    status = read_magneto( magneto );
		    if( status != 0 )
		    {
			Serial.println( "Could not read magnetometer" );
			sensor_fails[0] += 1;
			if( sensor_fails[0] >= SENSOR_FAIL_THRESHOLD )
			    Serial.println( "Magnetometer reached the sensor fail threshold" );
			continue;
		    }
		    Serial.println( "Magneto success" );
		    
		    resp.sensor = PERI_MAGNETO;
		    resp.data.magneto = magneto;
		    xQueueSendToBack( uart_out_drq, &resp, TICKS_TO_WAIT );
		    
		    Serial.printf( "Magneto x: %i\n", magneto.x );
		    Serial.printf( "Magneto y: %i\n", magneto.y );
		    Serial.printf( "Magneto z: %i\n", magneto.z );
		    Serial.printf( "Magneto head: %i\n", magneto.head );
		}
	    }
	    else
	    {
		qmc_up = qmc.begin();
		if( qmc_up )
		{
		    Serial.println( "Successfully started QMC" );
		    sensor_fails[0] = 0;
		}
		else	    
		    Serial.println( "Failed to start QMC" );
	    }
	    break;
	    
	case PERI_ACCEL:
	    if( !bmi_err && sensor_fails[1] < SENSOR_FAIL_THRESHOLD )
	    {
	        status = read_bmi160_accel( accel );
		if( status != 0)
		{
		    Serial.printf( "Failed to read accelerometer, code: %d\n", status );
		    sensor_fails[1] += 1;
		    if( sensor_fails[1] >= SENSOR_FAIL_THRESHOLD )
			Serial.println( "Accelerometer reached the sensor fail threshold" );
		}

		resp.sensor = PERI_ACCEL;
		resp.data.accel = accel;
		xQueueSendToBack( uart_out_drq, &resp, TICKS_TO_WAIT );
		
	        Serial.printf( "ortho_x: %f\n", accel.x );
	        Serial.printf( "ortho_y: %f\n", accel.y );
	        Serial.printf( "ortho_z: %f\n", accel.z );
	        Serial.printf( "roll: %f\n", accel.roll );
	        Serial.printf( "pitch: %f\n\n", accel.pitch );
	    }
	    else
	    {
		bmi_err = bmi160_init();
		if( !bmi_err )
		{
		    Serial.println( "Successfully started BMI160" );
		    sensor_fails[1] = 0;
		}
		else
		    Serial.printf( "Failed to start BMI160, code: %d\n", bmi_err );
	    }
	

	    break;
	case PERI_GYRO:
	    if( !bmi_err && sensor_fails[1] < SENSOR_FAIL_THRESHOLD )
	    {
		status = read_bmi160_gyro( gyro );
		if( status != 0)
		{
		    Serial.printf( "Failed to read gyroscope, code: %d\n", status );
		    sensor_fails[1] += 1;
		    if( sensor_fails[1] >= SENSOR_FAIL_THRESHOLD )
			Serial.println( "Gyroscope reached the sensor fail threshold" );
		}

		resp.sensor = PERI_GYRO;
		resp.data.gyro = gyro;
		xQueueSendToBack( uart_out_drq, &resp, TICKS_TO_WAIT );
		
	        Serial.printf( "gyro_x: %f\n", gyro.x );
	        Serial.printf( "gyro_y: %f\n", gyro.y );
	        Serial.printf( "gyro_z: %f\n", gyro.z );
	    }
	    else
	    {
		bmi_err = bmi160_init();
		if( !bmi_err )
		{
		    Serial.println( "Successfully started BMI160" );
		    sensor_fails[1] = 0;
		}
		else
		    Serial.printf( "Failed to start BMI160, code: %d\n", bmi_err );
	    }
	

	    break;
	}
    }
}
