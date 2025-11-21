#include <SPI.h>
#include <Wire.h>

//#define I2C_ADDR NULL
//#define CS
//#define CIPO
//#define SCK
//#define COPI
#define COEFF_I	( 1 )
#define COEFF_D	( 1 )


#define ms / portTICK_PERIOD_MS

struct Pos {
    float x;
    float y;
    float heading;
};

int read_gps(Pos* pos) {
    
}

void RadioTask(void* params) {
    TickType_t prev_wake = xTaskGetTickCount();
    while(true) {
	vTaskDelayUntil(&prev_wake, 500 ms);
    }
}

void DeployTask(void* params) {
    while(true) {
	vTaskSuspend(NULL);
    }
    Serial.println("DeployTask exited");
    vTaskDelete(NULL);
}

void SensorTask(void* params) {
    //Temperature, Pressure
    while(true) {

    }
}

void MotorTask(void* params) {
    float integral = 0;
    float err, pid;
    struct Pos prev_pos, curr_pos;
    while(true) {
	int err = read_gps(&curr_pos);
	integral += err;
	pid = err + (COEFF_I * integral) + (COEFF_D * err * 1000 / delta_t);
    }
}

void setup() {
    /* SPI.begin(); */
    /* SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); */
    //Wire1.begin(I2C_ADDR);
    pinMode(2, OUTPUT);
    Serial.begin(115200);
    
    /* xTaskCreate( */
    /* 		PIDTask,         // Task function */
    /* 		"PIDTask",       // Task name */
    /* 		10000,             // Stack size (bytes) */
    /* 		NULL,              // Parameters */
    /* 		2,                 // Priority */
    /* 		NULL  // Task handle */
    /* 		); */
    /* xTaskCreate( */
    /* 	SerialTask,         // Task function */
    /* 	"SerialTask",       // Task name */
    /* 	30000,             // Stack size (bytes) */
    /* 	NULL,              // Parameters */
    /* 	1,                 // Priority */
    /* 	&SerialTaskHandle  // Task handle */
    /* 	); */
}
void loop() {}
