#define MS / portTICK_PERIOD_MS
#define QUEUE_LEN 64

enum I2CDataSource
{
    I2C_ACCEL,
    I2C_GPS
};
enum SpiDataSource
{
    SPI_BMP,
    SPI_RADIO,
    SPI_ORIENT
};
enum ChuteDataSource
{
    CHUTE_BMP,
    CHUTE_GPS,
    CHUTE_MAGNET,
    CHUTE_GYRO
};
enum CmdOpcode
{
    CMD_MOVE_MOTOR,
    
};
struct RadioCmd
{
    enum CmdOpcode opcode;
};

union RadioDataSource
{
    enum I2CDataSource i2c;
    enum SpiDataSource spi;
    struct RadioCmd cmd;
};
enum DataDest
{
    DEST_I2C,
    DEST_CHUTE,
    DEST_SPI,
    DEST_UART
};
struct SpiDataRequest
{
    enum DataDest dest;
    enum SpiDataSource type;
};
struct I2CDataRequest
{
    enum DataDest dest;
    enum I2CDataSource type;
};
struct ChuteDataRequest
{
    enum DataDest dest;
    enum ChuteDataSource type;
};

struct GPSData		{ float x, y; };
struct AccelData	{ float x, y, z; };
struct MagnetData	{ float orient; };
struct SDData		{};
struct BMPData		{ float temp, pressure; };

//If dest == DEST_UART, then interpret request as command
struct RadioRequest
{
    enum DataDest dest;
    union RadioDataSource;
};
struct RadioResponse	{};

union SpiDataContent
{
    struct BMPData bmp;
    struct
};
union I2CDataContent
{
    struct AccelData accel;
    struct MagnetData magnet;
};
union ChuteDataContent
{
    struct BMPData bmp;
    struct GPSData gps;
    struct MagnetData magnet;
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
