#define NUM_SOURCES	( 3 )

enum I2CDataSource
{
    I2C_ACCEL,
    I2C_GPS,
    I2C_MAGNETO
};
enum SpiDataSource
{
    SPI_BMP
};
enum ChuteDataSource
{
    CHUTE_BMP,
    CHUTE_GPS,
    CHUTE_MAGNET,
    CHUTE_GYRO
};

/* enum CmdOpcode */
/* { */
/*     CMD_MOVE_MOTOR, */
/*     CMD_GET_STATUS */
/* }; */
/* struct RadioCmd */
/* { */
/*     int32_t arg; */
/*     enum CmdOpcode opcode; */
/* }; */

enum DataSourceEnum
{
    SOURCE_SPI,
    SOURCE_I2C
};

union DataSourceUnion
{
    enum I2CDataSource i2c;
    enum SpiDataSource spi;
};
struct DataSource
{
    enum DataSourceEnum denum;
    union DataSourceUnion dunion;
};

enum DataDest
{
    DEST_I2C,
    DEST_CHUTE,
    DEST_SPI,
    DEST_UART
};

//Comes from ground station
struct RadioRequest
{
    enum DataDest dest;
    union DataSourceUnion source; 
};

struct GPSData		{ float x, y; };
struct AccelData
{
    float ortho_x;
    float ortho_y;
    float ortho_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
};
struct MagnetoData	{ float x, y, z, head; };
struct SDData		{};
struct BMPData		{ float temp, pressure; };
struct GyroData		{};

//Comes from sat
struct RadioResponse	{};

union SpiDataContent
{
    struct BMPData bmp;
    /* struct  */
};
union I2CDataContent
{
    struct AccelData accel;
    struct MagnetoData magneto;
};
union ChuteDataContent
{
    struct BMPData bmp;
    struct GPSData gps;
    struct MagnetoData magneto;
    struct GyroData gyro;
};

struct SpiData
{
    enum SpiDataSource source;
    union SpiDataContent content;
};
struct I2CData
{
    enum I2CDataSource source;
    union I2CDataContent content;
};
struct ChuteData
{
    enum ChuteDataSource source;
    union ChuteDataContent content;
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

#define QMC_FREQ_REG_ADDR		( 0x00 )
#define QMC_GAIN_REG_ADDR		( 0x01 )
#define QMC_MODE_REG_ADDR		( 0x02 )

#define QMC_FREQ_REG_VALUE		( 0x70 )
#define QMC_GAIN_REG_VALUE		( 0xA0 )
#ifdef QMC_SINGLE_READ
#define QMC_MODE_REG_VALUE		( 0x01 )
#else
#define QMC_MODE_REG_VALUE		( 0x00 )
#endif
/* #define QMC_ */

#define I2C_BMP_ADDR			( 0x76 )
#define I2C_BMI_ADDR			( 0x69 )
#define I2C_QMC_ADDR			( 0x2C )

#define MS				/ portTICK_PERIOD_MS

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} hmc5883l_data_t;

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
