// Pins numbers are according to IOxyz

// SPI
#define SPI_MISO 			( 19 ) //BMP: SDO/SDD
#define SPI_MOSI 			( 23 ) //BMP: SDA
#define SPI_SCK 			( 18 ) //BMP: SCL

#define SPI_BMP_SS			( 15 ) //BMP: CSB
#define SPI_SD_SS			( 16 )

// I2C
#define I2C_SDA				( 21 )
#define I2C_SCL				( 22 )
#define I2C_FREQ			( 1e5 )

// BMP280
#define BMP_RES_BITS			( 16 )

// QMC5883P
#define QMC_SINGLE_READ
#define QMC_DECL_ANGLE			( 5.75090 )
#define QMC_OFFSET_X			( 0.751 )
#define QMC_OFFSET_Y			( 0.038 )
#define QMC_SCALE_X			( 0.390 )
#define QMC_SCALE_Y			( 0.414 )
#define QMC_SCALE_AVG			( 0.402 )
#define QMC_ANGLE_OFFSET		( 210.0 )

// Guidance control
#define PN_GAIN				( 3.0 )   // Proportional navigation gain (typically 3-5)
#define STEERING_GAIN			( 0.5 )   // Scales heading error to motor output
#define MOTOR_DEADBAND			( 10.0 )  // Ignore heading errors below this (degrees)
#define MOTOR_MAX_PWM			( 255 )   // Max PWM value for motor

// UART
#define UART_RADIO_BAUD			( 115200 )
#define UART_RADIO_RX 			( 16 )
#define UART_RADIO_TX 			( 17 )
#define UART_RADIO_INVERT		( false )
#define UART_RADIO_TIMEOUT_MS		( 10 )
#define UART_RADIO_RXFIFO_FULL_THRHD_MS	( 100 )

#define UART_SERIAL_BAUD		( 115200 )
#define UART_SERIAL_RX 			( 3 )
#define UART_SERIAL_TX 			( 1 )

#define UART_RX_BUF_SIZE		( 1024 )
#define UART_TX_BUF_SIZE		( 1024 )

// Radio
#define RADIO_FREQ			( 868e6 )

#define PACKET_SYNC_BYTE    0xAA
#define PACKET_HEADER_SIZE  7   // sync(1) + type(1) + ticks(4) + len(1)
#define PACKET_CRC_SIZE     4

// Miscellaneous
#define PRINT_TO_SERIAL			( false )

#define SPI_DRQ_LEN			( 64 )
#define I2C_DRQ_LEN			( 64 )
#define CHUTE_DRQ_LEN			( 64 )
#define UART_OUT_LEN			( 64 )

#define DEFAULT_DATA_REQUEST_INTERVAL	( 10000 MS )
#define TICKS_TO_WAIT			( 10 )
#define SENSOR_FAIL_THRESHOLD		( 10 )

/* #define TEST_PIN			( 33 ) */
/* #define STREAM_BUF_LEN			( 1024 ) */
/* #define STREAM_BUF_TRIG			( 4 ) */
