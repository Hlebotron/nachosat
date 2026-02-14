/*!
 * @brief  		Read the data of the BMP280 and write it to the parameter "data"
 * @param bmp_temp	The sensor object used to read the temperature
 * @param bmp_pressure	The sensor object used to read the pressure
 * @param data		The struct where the data of the BMP280 is stored after the read
 * @returns
 * 	0 if OK
 *	1 if bmp_temp is NULL
 *	2 if bmp_pressure is NULL
 */
int32_t read_bmp( Adafruit_Sensor* bmp_temp, Adafruit_Sensor* bmp_pressure, struct BMPData& data );


/*!
 * @brief		The task responsible for communicating using SPI
 * @param params	Mandatory parameter of a FreeRTOS task, use for signaling task state
 */
void SpiTask( void* params );
