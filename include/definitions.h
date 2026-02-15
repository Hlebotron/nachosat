#define NUM_SOURCES	( 3 )

enum Interface
{
    IF_I2C,
    IF_SPI,
    IF_CHUTE,
    IF_UART
};

enum Peripheral
{
    //I2C devices
    PERI_ACCEL,	
    PERI_GYRO,
    PERI_GPS,
    PERI_MAGNETO,
    
    //SPI devices
    PERI_BMP,
    PERI_SD
};

/* //Comes from ground station */
/* struct RadioRequest */
/* { */
    
/* }; */

struct GPSData		{ float x, y; };
struct MagnetoData	{ float x, y, z, head; };
struct BMPData		{ float temp, pressure; };
struct AccelData 	{ float x, y, z, roll, pitch; };
struct GyroData 	{ float x, y, z; };

union PeriData
{
    struct GPSData	gps;
    struct AccelData	accel;
    struct GyroData	gyro;
    struct MagnetoData	magneto;
    struct BMPData	bmp;
    uint8_t		byte;
};

//Comes from sat
struct RadioResponse
{
    enum Peripheral	sensor;
    union PeriData	data;
};

enum i2c_error
{
    I2C_OK = 0,
    I2C_ERR_NACK_ADDR_TRANSMIT = 2,
    I2C_ERR_NACK_DATA_TRANSMIT = 3,
    I2C_ERR_LINE_BUSY = 4
};

#define BMI_CHIP_ID   			( 0x00 )
#define BMI_CMD       			( 0x7E )

#define BMI_ACC_CONF  			( 0x40 )
#define BMI_ACC_RANGE 			( 0x41 )
#define BMI_GYR_CONF  			( 0x42 )
#define BMI_GYR_RANGE 			( 0x43 )

#define BMI_GYR_X_L   			( 0x0C )
#define BMI_ACC_X_L   			( 0x12 )

#define BMP_TEMP			( 0xFA )
#define BMP_PRESS			( 0xF7 )
#define BMP_MIN_RES_TEMP		( 0.0050f )
#define BMP_MAX_RES_PRESS		( 0.16f )
#define BMP_MIN_RES_BITS		( 16 )
#define BMP_MAX_RES_BITS		( 20 )

#define QMC_SIGN_REG_ADDR		( 0x29 )
/* #define QMC_FREQ_REG_ADDR		( 0x00 ) */
/* #define QMC_GAIN_REG_ADDR		( 0x01 ) */
/* #define QMC_MODE_REG_ADDR		( 0x02 ) */
#define QMC_RS_REG_ADDR			( 0x0B )
#define QMC_MODE_REG_ADDR		( 0x0A )

#define QMC_SIGN_REG_VALUE		( 0x06 )
#define QMC_RS_REG_VALUE		( 0x08 )
/* #define QMC_FREQ_REG_VALUE		( 0x70 ) */
/* #define QMC_GAIN_REG_VALUE		( 0xA0 ) */
#ifdef QMC_SINGLE_READ
/* #define QMC_MODE_REG_VALUE		( 0x01 ) */

#define QMC_MODE_REG_VALUE		( 0xCD )
#else
/* #define QMC_MODE_REG_VALUE		( 0x00 ) */
#define QMC_MODE_REG_VALUE		( 0xC3 )
#endif
/* #define QMC_ */

#define I2C_BMP_ADDR			( 0x76 )
#define I2C_BMI_ADDR			( 0x69 )
#define I2C_QMC_ADDR			( 0x2C )

#define MS				/ portTICK_PERIOD_MS

/* typedef struct { */
/*     int16_t x; */
/*     int16_t y; */
/*     int16_t z; */
/* } hmc5883l_data_t; */

/* #ifndef QMC5883L_H */
/* #define QMC5883L_H */

/* #include <stdint.h> */
/* #include <stdbool.h> */

/* // I2C address */
/* #define QMC5883L_ADDR 0x1E */

/* // Registers */
/* #define QMC5883L_REG_CONFIG_A   0x00 */
/* #define QMC5883L_REG_CONFIG_B   0x01 */
/* #define QMC5883L_REG_MODE       0x02 */
/* #define QMC5883L_REG_DATA_X_MSB 0x03 */

/* // Modes */
/* #define QMC5883L_MODE_CONTINUOUS 0x00 */

/* typedef struct { */
/*     int16_t x; */
/*     int16_t y; */
/*     int16_t z; */
/* } hmc5883l_data_t; */

/* // Public API */
/* bool hmc5883l_init(); */
/* bool hmc5883l_read(hmc5883l_data_t *out); */

/* // ISR hook */
/* void IRAM_ATTR hmc5883l_drdy_isr(); */

/* #endif */
