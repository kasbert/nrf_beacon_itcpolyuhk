// Sensor values
typedef struct ruuvi_sensor_t
{
  uint8_t     format;         // 0x03
  uint8_t     humidity;       // one lsb is 0.5%
  uint16_t    temperature;    // Signed 8.8 fixed-point notation.
  uint16_t    pressure;       // Pascals (pa)
  int16_t     accX;           // Milli-g (mg)
  int16_t     accY;
  int16_t     accZ;
  uint16_t    vbat;           // mv
} ruuvi_sensor_t;

void parseSensorData(struct ruuvi_sensor_t* data, int32_t raw_t, uint32_t raw_p, uint32_t raw_h, uint16_t vbat, int32_t acc[3]);
void encodeToUrlDataFromat(char* buf, ruuvi_sensor_t* data);
