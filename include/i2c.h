#include <Wire.h>

/*!
  @brief 	Write a byte to a register of the BMI160
  @param reg 	Address of the register
  @param data	The byte to send
  @return	0: 	OK
  		-1: 	Wire.write() failed at reg
		-2: 	Wire.write() failed at data
		1: 	data too long to fit in transmit buffer (Wire.endTransmission)
		2: 	received NACK on transmit of address (Wire.endTransmission)
		3: 	received NACK on transmit of data (Wire.endTransmission)
		4: 	other error (Wire.endTransmission)
		5: 	timeout (Wire.endTransmission)
 */
int bmi160_write( uint8_t reg, uint8_t data );

/*!
  @brief 	Read a byte from a register of the BMI160
  @param reg 	Address of the register
  @param data	The byte to store the contents in
  @return	 0:	OK
  		-1:	Wire.write() failed
  		-2:	Wire.requestFrom() failed
		1:	data too long to fit in transmit buffer (Wire.endTransmission)
		2:	received NACK on transmit of address (Wire.endTransmission)
		3:	received NACK on transmit of data (Wire.endTransmission)
		4:	other error (Wire.endTransmission)
		5:	timeout (Wire.endTransmission)
 */
int bmi160_read8( uint8_t reg, uint8_t& out );

/*!
  @brief 	Read 2 bytes in total from 2 registers of the BMI160
  @param reg 	Address of the first register
  @param data	The bytes to store the contents in
  @return	0:	OK
		Everything else is the result of bmi_read8()
 */
int bmi160_read16( uint8_t reg, int16_t& out );

/*!
  @brief 	Initialize the BMI160
  @return	 0:	OK
  		-3:	assertion id != 0xD1 failed
		-4:	Wire.write() failed (bmi160_read8)
  		-5:	Wire.requestFrom() failed (bmi160_read8)
		Everything else is the result of write_reg2()
 */
int bmi160_init();

/*!
  @brief 	Read the acceleration data of the BMI160
  @param accel 	Accelerometer struct to store the data in
  @return	0:	OK

		Everything else is the result of bmi160_read16()
 */
int read_bmi160_accel( AccelData& accel );

/*!
  @brief 	Read the gyroscope data of the BMI160
  @param gyro 	Gyroscope struct to store the data in
  @return	0:	OK
		Everything else is the result of bmi160_read16()
 */
int read_bmi160_gyro( GyroData& gyro );

/*!
  @brief 		Write 1 byte to an I2C device
  @param dev_addr	I2C address of the device being written to
  @param val		Value to write
    @return		0: 	OK
  			-1: 	failed to write addr (Wire.write)
			-2: 	failed to write val (Wire.write)
			1: 	data too long to fit in transmit buffer (Wire.endTransmission)
			2: 	received NACK on transmit of address (Wire.endTransmission)
			3: 	received NACK on transmit of data (Wire.endTransmission)
			4: 	other error (Wire.endTransmission)
			5: 	timeout (Wire.endTransmission)
 */
int write_reg1( uint8_t dev_addr, uint8_t val, bool endstop );

/*!
  @brief 		Write 2 bytes to a register of an I2C device
  @param dev_addr	I2C address of the device being written to
  @param addr		Address of register
  @param val		Value to write
  @return		0: 	OK
  			-1: 	failed to write addr (Wire.write)
			-2: 	failed to write val (Wire.write)
			1: 	data too long to fit in transmit buffer (Wire.endTransmission)
			2: 	received NACK on transmit of address (Wire.endTransmission)
			3: 	received NACK on transmit of data (Wire.endTransmission)
			4: 	other error (Wire.endTransmission)
			5: 	timeout (Wire.endTransmission)
 */
int write_reg2( uint8_t dev_addr, uint8_t addr, uint8_t val, bool endstop );

/*!
  @brief 		Initialize the QMC5883P
  @return		0: 	OK
  			Everything else is a result of write_reg2()

 */
int magneto_init();

/*!
  @brief 		Read the QMC5883P
  @param magneto	Where the magnetometer data will be stored
  @return		0: 	OK
  			-3: 	Wire.requestFrom() failed
			-4:	Wire.available() failed (no data available)
			Everything else is the result of write_reg1() and write_reg2()
 */
int read_magneto( MagnetoData& magneto );

/*!
  @brief 		FreeRTOS task responsible for communicating using I2C
  @param params		Mandatory parameter of a FreeRTOS task, used for signaling state
 */
void I2CTask( void* params );
