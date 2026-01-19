#define QUEUE_LEN 		( 64 )

#define MISO 			( 19 ) //BMP: SDO/SDD
#define MOSI 			( 23 ) //BMP: SDA
#define SCK 			( 18 ) //BMP: SCL
#define BMP_SS 			( 15 ) //BMP: CSB
#define SD_SS			( 16 )


#define BMP_MIN_RES_BITS	( 16 )
#define BMP_MAX_RES_BITS	( 20 )

#define BMP_RES_BITS		( 16 )

#define HMC_SINGLE_READ

#define I2C_SDA			( 21 )
#define I2C_SCL			( 22 )
#define I2C_FREQ		( 100000 )

#define PID_COEFF_P		( 1 )
#define PID_COEFF_I		( 1 )
#define PID_COEFF_D		( 1 )

#define UART_BAUD 		( 115200 )
#define COMMS_UART_RX 		( 16 )
#define COMMS_UART_TX 		( 17 )
