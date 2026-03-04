#include <HardwareSerial.h>
#include <Wire.h>
#include "i2c.h"
#include <math.h>
// extern SemaphoreHandle_t received_sem = xSemaphoreCreateBinary();
// extern QueueHandle_t i2c_drq;

static uint8_t sensor_fails[ NUM_SOURCES ]; // = { 0, 0, 0 }; // BMI160, QMC, GPS

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

// ========================= GPS (MAX-M8Q) =========================

// Read available bytes from GPS DDC stream
static int gps_read_available()
{
    Wire.beginTransmission(I2C_GPS_ADDR);
    Wire.write(0xFD);  // Register for bytes available (high byte)
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(I2C_GPS_ADDR, 2) != 2) return -2;
    uint16_t available = (Wire.read() << 8) | Wire.read();
    return available;
}

// Read bytes from GPS DDC data stream
static int gps_read_bytes(uint8_t* buf, size_t len)
{
    Wire.beginTransmission(I2C_GPS_ADDR);
    Wire.write(0xFF);  // Data stream register
    if (Wire.endTransmission(false) != 0) return -1;

    size_t received = Wire.requestFrom(I2C_GPS_ADDR, len);
    if (received != len) return -2;

    for (size_t i = 0; i < len; i++)
    {
        buf[i] = Wire.read();
    }
    return 0;
}

// Calculate UBX checksum (Fletcher's algorithm)
static void ubx_checksum(const uint8_t* data, size_t len, uint8_t* ck_a, uint8_t* ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++)
    {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

// Read GPS position from UBX-NAV-PVT message
// Returns 0 on success, negative on error, positive if no valid fix
int read_gps(GPSData& gps)
{
    // Check if data is available
    int available = gps_read_available();
    if (available < 0) return available;
    if (available < 100) return 1;  // Not enough for a full NAV-PVT message

    // Buffer for UBX-NAV-PVT: sync(2) + class(1) + id(1) + len(2) + payload(92) + cksum(2) = 100 bytes
    uint8_t buf[100];

    // Search for UBX sync bytes in available data
    bool found = false;
    for (int attempts = 0; attempts < available && !found; attempts++)
    {
        uint8_t byte;
        if (gps_read_bytes(&byte, 1) != 0) return -3;

        if (byte == UBX_SYNC_1)
        {
            buf[0] = byte;
            if (gps_read_bytes(&buf[1], 1) != 0) return -4;

            if (buf[1] == UBX_SYNC_2)
            {
                // Read rest of header (class, id, length)
                if (gps_read_bytes(&buf[2], 4) != 0) return -5;

                uint8_t msg_class = buf[2];
                uint8_t msg_id = buf[3];
                uint16_t payload_len = buf[4] | (buf[5] << 8);

                // Check if this is NAV-PVT
                if (msg_class == UBX_NAV_CLASS && msg_id == UBX_NAV_PVT_ID && payload_len == UBX_NAV_PVT_LEN)
                {
                    // Read payload + checksum
                    if (gps_read_bytes(&buf[6], payload_len + 2) != 0) return -6;
                    found = true;
                }
            }
        }
    }

    if (!found) return 2;  // No NAV-PVT message found

    // Verify checksum (covers class, id, length, payload)
    uint8_t ck_a, ck_b;
    ubx_checksum(&buf[2], 4 + UBX_NAV_PVT_LEN, &ck_a, &ck_b);
    if (ck_a != buf[98] || ck_b != buf[99]) return 3;  // Checksum mismatch

    // Check fix type (offset 20 in payload, which is buf[26])
    uint8_t fix_type = buf[26];
    if (fix_type < 2) return 4;  // No fix (0=none, 1=dead reckoning only)

    // Extract lon/lat from payload
    // Payload starts at buf[6]
    // lon @ offset 24 in payload = buf[6+24] = buf[30]
    // lat @ offset 28 in payload = buf[6+28] = buf[34]
    int32_t lon_raw = buf[30] | (buf[31] << 8) | (buf[32] << 16) | (buf[33] << 24);
    int32_t lat_raw = buf[34] | (buf[35] << 8) | (buf[36] << 16) | (buf[37] << 24);

    // Convert from 1e-7 degrees to float degrees
    gps.lon = lon_raw / 10000000.0f;
    gps.lat = lat_raw / 10000000.0f;

    return 0;
}

void I2CTask( void* params )
{
    // Wire.begin( I2C_SDA, I2C_SCL, I2C_FREQ );
    memset( sensor_fails, 0, NUM_SOURCES );

  
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
    GPSData gps;
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
		status = qmc.readXYZ( xyz );
		if( !status )
		{
		    Serial.println( "Could not read magnetometer xyz" );
		    sensor_fails[0] += 1;
		    if( sensor_fails[0] >= SENSOR_FAIL_THRESHOLD )
			Serial.println( "Magnetometer reached the sensor fail threshold" );
		    continue;
		}

		magneto.head = qmc.getHeadingDeg( QMC_DECL_ANGLE );
		    
		resp.sensor = PERI_MAGNETO;
		resp.data.magneto = magneto;
		xQueueSendToBack( uart_out_drq, &resp, TICKS_TO_WAIT );
		    
		Serial.printf( "Magneto x: %f\n", xyz[0] );
		Serial.printf( "Magneto y: %f\n", xyz[1] );
		Serial.printf( "Magneto z: %f\n", xyz[2] );
		Serial.printf( "Magneto head: %f\n", magneto.head );
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
		xTaskNotifyGive( uart_handle );
		
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

	case PERI_GPS:
	    if (sensor_fails[2] < SENSOR_FAIL_THRESHOLD)
	    {
		status = read_gps(gps);
		if (status < 0)
		{
		    Serial.printf("Failed to read GPS, code: %d\n", status);
		    sensor_fails[2] += 1;
		    if (sensor_fails[2] >= SENSOR_FAIL_THRESHOLD)
			Serial.println("GPS reached the sensor fail threshold");
		}
		else if (status > 0)
		{
		    Serial.printf("GPS: no valid data (code %d)\n", status);
		}
		else
		{
		    resp.sensor = PERI_GPS;
		    resp.data.gps = gps;
		    xQueueSendToBack(uart_out_drq, &resp, TICKS_TO_WAIT);

		    Serial.printf("GPS lat: %f\n", gps.lat);
		    Serial.printf("GPS lon: %f\n", gps.lon);
		}
	    }
	    break;
	}
    }
}
