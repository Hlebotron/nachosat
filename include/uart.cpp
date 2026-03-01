extern TaskHandle_t uart_handle;

#ifndef GROUND
extern QueueHandle_t uart_out_drq;
#endif

void uart_isr()
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    // xTaskNotifyFromISR( uart_handle, 1, eSetBits, &higher_priority_task_woken ); // Actually notify the task
    vTaskNotifyGiveFromISR( uart_handle, &higher_priority_task_woken ); // Actually notify the task
    // vTaskNotifyGiveIndexedFromISR( uart_handle, 2, &higher_priority_task_woken2 ); // Notify the task about a message from the radio
    portYIELD_FROM_ISR( higher_priority_task_woken );
}

int write_radio( HardwareSerial& Radio, const RadioResponse* resp )
{
    static size_t written = 0;
    if( resp == NULL )
	return 2;
    
    switch( resp->sensor )
    {
    case PERI_ACCEL:
	written = Radio.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(AccelData)) );
	break;
    case PERI_GYRO:
	written = Radio.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(GyroData)) );
	break;
    case PERI_GPS:
	written = Radio.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(GPSData)) );
	break;
    case PERI_MAGNETO:
	written = Radio.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(MagnetoData)) );
	break;
    case PERI_BMP:
	written = Radio.write( (uint8_t*) resp, (sizeof(Peripheral) + sizeof(BMPData)) );
	break;
    default:
	Serial.printf( "Invalid sensor, code: %i\n", resp->sensor );
	return 1;
    }
    
    if( written == 0 )
	return 3;
    
    return 0;
}

int radio_init( HardwareSerial& Radio )
{
    Radio.begin( UART_RADIO_BAUD, SERIAL_8N1, UART_RADIO_RX, UART_RADIO_TX );
    attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_isr, FALLING );

    Radio.write( "radio rxstop\r\n" ); 			delay( 500 );
    Radio.write( "radio set freq" );
    Radio.print( RADIO_FREQ );
    Radio.write( "\r\n" );				delay( 500 );
    Radio.write( "radio set mod lora\r\n" ); 		delay( 500 );
    Radio.write( "radio set sf sf7\r\n" ); 		delay( 500 );
    Radio.write( "radio set bw 125\r\n" ); 		delay( 500 );
    Radio.write( "radio set pa off\r\n" ); 		delay( 500 );
    Radio.write( "radio set pwr 10\r\n" ); 		delay( 500 );
    Radio.write( "radio rxstop\r\n" ); 			delay( 500 );
    Radio.write( "radio rx 0\r\n" ); 			delay( 500 );
    return 0;
}

#ifdef GROUND

void UartTask( void* params )
{
    attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_isr, FALLING );
    attachInterrupt( digitalPinToInterrupt(UART_SERIAL_RX), uart_isr, FALLING );
    HardwareSerial Radio( 1 );
    
    Serial.begin( UART_SERIAL_BAUD );
    Radio.begin( UART_RADIO_BAUD, SERIAL_8N1, UART_RADIO_RX, UART_RADIO_TX, UART_RADIO_INVERT, UART_RADIO_TIMEOUT_MS, UART_RADIO_RXFIFO_FULL_THRHD_MS );

    Serial.println( "Starting" );
    for( ;; )
    {
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); // Get notified about either the radio or serial

	if( Radio.available() )
	{
	    Serial.print( "UART data available: " );
	    while( Radio.available() )
	    {
		Serial.printf( "%c", Radio.read() );
	    }
	    Serial.println();
	}

	if( Serial.available() )
	{
	    Serial.print( "Serial data available " );
	    while( Serial.available() )
	    {
	    #if PRINT_TO_SERIAL
		Serial.println( Serial.read() );
	    #else
		Radio.write( Serial.read() );
	    #endif
		Serial.println();
	    }
	}
	
	vTaskDelay( 3 );
    }
}


#else

void UartTask( void* params )
{
    attachInterrupt( digitalPinToInterrupt(UART_RADIO_RX), uart_isr, FALLING );
    attachInterrupt( digitalPinToInterrupt(UART_SERIAL_RX), uart_isr, FALLING );
    HardwareSerial Radio( 1 );
    
    // //Create the queue set
    // static QueueSetHandle_t queue_set = xQueueCreateSet( QUEUE_LEN + 1 ); //Length of data_response_queue + serial_sem
    // xQueueAddToSet( serial_sem, queue_set );
    // xQueueAddToSet( queue_from_uart, queue_set );
#if !PRINT_TO_SERIAL
    radio_init( Radio );
#endif
    
    Serial.begin( UART_SERIAL_BAUD );
    // Radio.begin( UART_RADIO_BAUD );
    // Radio.begin( UART_RADIO_BAUD, SERIAL_8N1, UART_RADIO_RX, UART_RADIO_TX, false, 10, 100 );
    Radio.begin( UART_RADIO_BAUD, SERIAL_8N1, UART_RADIO_RX, UART_RADIO_TX, UART_RADIO_INVERT, UART_RADIO_TIMEOUT_MS, UART_RADIO_RXFIFO_FULL_THRHD_MS );

    // radio_init( Radio );

    // *( BaseType_t* ) params = 3;

    size_t radio_writable = 0;
    RadioResponse resp;
    
    Serial.println( "Starting" );
    for( ;; )
    {
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); // Get notified about either the radio or serial

	if( Radio.available() )
	{
	    Serial.print( "UART data available: " );
	    while( Radio.available() )
	    {
		Serial.printf( "%c", Radio.read() );
	    }
	    Serial.println();
	}

	if( xQueueReceive(uart_out_drq, &resp, 0) == pdPASS )
	{
	    do
	    {
		Serial.printf( "Got data, type: %d\n", resp.sensor );
		write_radio( (PRINT_TO_SERIAL ? Serial : Radio), &resp);
	    }
	    while( xQueueReceive(uart_out_drq, &resp, 0) == pdPASS );
	}
	
	if( Serial.available() )
	{
	    Serial.print( "Serial data available " );
	    // radio_writable = Radio.availableForWrite();
	    // if( radio_writable >= sizeof("Hello there") )
	    // 	Radio.write( "Hello there" );
	    // else
	    // 	Serial.println( "Unable to print Hello There" );
		
		
	    // while( Serial.available() )
	    // 	Serial.read();
	    {
	    #if PRINT_TO_SERIAL
		Serial.println( Serial.read() );
	    #else
		Radio.write( Serial.read() );
	    #endif
		Serial.println();
		// Serial.println( "\nSent Hello There" );
	    }
	}

	
	vTaskDelay( 3 );
    }
}

// void UartTask( void* params )
// {
//     HardwareSerial Radio( 1 );
//     radio_init( Radio );

//     *( BaseType_t* ) params = 3;

    
//     RadioResponse resp;
//     for( ;; )
//     {
// 	member = xQueueSelectFromSet( uart_set, ( 1000 MS ) );
// 	if( member == uart_in_sem ) //If a message has arrived
// 	{
// 	    xSemaphoreTake( uart_in_sem, TICKS_TO_WAIT );
	    
// 	    while( Radio.available() )
// 	    {
// 		Serial.print( Radio.read() );
// 	    }
// 	}
// 	else if( member == uart_out_drq ) //If we want to send something
// 	{
// 	    BaseType_t res = xQueueReceive( uart_out_drq, (void*) &resp, TICKS_TO_WAIT );
// 	    write_radio( Radio, &resp );
// 	}
// 	else
// 	{
// 	    Serial.println( "Invalid queue, unable to determine name" );
// 	    Serial.println( "Sending bogus data ");
// 	    Radio.println( "Hello there" );
// 	    Serial.println( "Sent bogus data ");
// 	}
//     }
// }

#endif
