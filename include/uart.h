/*!
  @brief		Write a block of data using the radio
  @param radio_uart	HardwareSerial object representing the radio
  @param resp		Content being written
  @return		0 if OK
  			1 if invalid sensor in resp.sensor
  			2 if resp == NULL
			3 if written == 0
 */
int write_radio( HardwareSerial& radio_uart, const RadioResponse* resp );

/*!
  @brief 		Initializes the radio
  @param radio_uart	HardwareSerial object used for the radio
  @return		TODO: Add return to the function
 */
int radio_init( HardwareSerial& radio_uart );
/* int read_radio(  ); */

/*!
  @brief 		Interrupt handler for UART RX, unblocks UART Task
 */
void uart_irq();

/*!
  @brief 		FreeRTOS task responsible for communicating using UART
  @param params		Mandatory parameter of a FreeRTOS task, used for signaling state
 */
void UartTask( void* params );
