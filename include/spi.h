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
int read_bmp( Adafruit_Sensor* bmp_temp, Adafruit_Sensor* bmp_pressure, struct BMPData& data );

/*!
 * @brief  	Read the data of the SD card and write it to the parameter "data"
 * @param data	The struct where the data of the SD card is stored after the read
 * @returns	TODO
 */
int read_sd( struct SDData& data );

/*!
 * @brief  	Read the data from the parameter "data" and write it to the SD card
 * @param data	The struct where the data of "data" is stored after the read
 * @returns	TODO
 */
int write_sd( const struct SDData& data );

/*!
 * @brief	The task responsible for communicating using SPI
 * @param params	Parameter required to create a FreeRTOS task, use for determining task state
 */
void SpiTask( void* params );
