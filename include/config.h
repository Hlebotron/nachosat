#define SPI_MISO 			( 19 ) //BMP: SDO/SDD
#define SPI_MOSI 			( 23 ) //BMP: SDA
#define SPI_SCK 			( 18 ) //BMP: SCL

#define SPI_BMP_SS			( 15 ) //BMP: CSB
#define SPI_SD_SS			( 16 )

#define I2C_SDA				( 21 )
#define I2C_SCL				( 22 )
#define I2C_FREQ			( 1e5 )

#define SPI_DRQ_LEN			( 64 )
#define I2C_DRQ_LEN			( 64 )
#define CHUTE_DRQ_LEN			( 64 )
#define UART_OUT_LEN			( 64 )

#define BMP_RES_BITS			( 16 )

#define QMC_SINGLE_READ
#define QMC_DECL_ANGLE			( 5.75090 )
#define QMC_OFFSET_X			( 0.751 )
#define QMC_OFFSET_Y			( 0.038 )
#define QMC_SCALE_X			( 0.390 )
#define QMC_SCALE_Y			( 0.414 )
#define QMC_SCALE_AVG			( 0.402 )
#define QMC_ANGLE_OFFSET		( 210.0 )

#define PID_COEFF_P			( 1 )
#define PID_COEFF_I			( 1 )
#define PID_COEFF_D			( 1 )

#define UART_RADIO_BAUD			( 115200 )
#define UART_RADIO_RX 			( 16 )
#define UART_RADIO_TX 			( 17 )

#define RADIO_FREQ			( 868e6 )

#define DEFAULT_DATA_REQUEST_INTERVAL	( 5000 MS )
#define TICKS_TO_WAIT			( 10 )
