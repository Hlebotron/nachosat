#include <Arduino.h>
#include <HardwareSerial.h>
// #include <cbuf.cpp>

#define GROUND

TaskHandle_t uart_handle; 

#include "config.h"
#include "definitions.h"
#include "uart.cpp"

void IdleHook()
{
    esp_sleep_enable_uart_wakeup( 1 );
    for( ;; )
    {
	esp_light_sleep_start();
    }
}


void setup()
{
    xTaskCreate( UartTask, "UartTask", 10000, NULL, 1, &uart_handle );
}

void loop() {}
