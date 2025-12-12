enum I2CDataSource
{
    I2C_BMP,
    I2C_BMP_TEMP,
    I2C_BMP_PRESS,
    I2C_ACCEL,
    I2C_GPS
};
enum SpiDataSource
{
    SPI_RADIO,
    SPI_ORIENT
};
enum ChuteDataSource
{
    CHUTE_BMP,
    CHUTE_GPS,
    CHUTE_ORIENT
};

struct GPSData		{ float x, y; };
struct AccelData	{ float x, y, z; };
struct OrientData	{ float orient; };
struct RadioData	{};
struct BMPData		{ float temp, pressure; };

union SpiDataContent
{
    struct GPSData gps;
    struct RadioData radio;
    struct OrientData orient;
};
union I2CDataContent
{
    float bmp;
    struct AccelData accel;
};
union ChuteDataContent
{
    float press;
    struct GPSData gps;
    struct OrientData orient;
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
