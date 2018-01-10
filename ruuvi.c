
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ruuvi.h"

/*
  0:   uint8_t     format;          // (0x03 = realtime sensor readings base64)
  1:   uint8_t     humidity;        // one lsb is 0.5%
  2-3: uint16_t    temperature;     // Signed 8.8 fixed-point notation.
  4-5: uint16_t    pressure;        // (-50kPa)
  6-7:   int16_t   acceleration_x;  // mg
  8-9:   int16_t   acceleration_y;  // mg
  10-11: int16_t   acceleration_z;  // mg
  12-13: int16_t   vbat;            // mv
*/
#define SENSOR_TAG_DATA_FORMAT          0x03                              /**< raw binary, includes acceler
  ation */
#define SENSORTAG_ENCODED_DATA_LENGTH   14            /* 14 bytes  */

#define WEATHER_STATION_URL_FORMAT      0x02                              /**< Base64 */
#define WEATHER_STATION_URL_ID_FORMAT   0x04                              /**< Base64, with ID byte */


#define EDDYSTONE_URL_MAX_LENGTH 17
#define URL_PAYLOAD_LENGTH 9

#define URL_BASE_MAX_LENGTH (EDDYSTONE_URL_MAX_LENGTH - URL_PAYLOAD_LENGTH)


/**
    Encodes a stream of binary data into base64 ascii string.
    endocoding efficiency is 75%, i.e. you need 4 chars (32 bits) to represent 24 bits of data
    Please note the implementation uses "-", "_" and "." as characters instead of standard
    "+". "/", "=" for url-safety

    @param data_buf data to encode
    @param dataLength length of data_buf
    @param result result buffer
    @param resultSize size of result buffer
    @return 1 on error, 0 on success.
*/
int base64encode(const void* data_buf, size_t dataLength, char* result, size_t resultSize)
{
  const char base64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  const char base64pad = '.';
  const uint8_t *data = (const uint8_t *)data_buf;
  size_t resultIndex = 0;
  size_t x;
  uint32_t n = 0;
  int padCount = dataLength % 3;
  uint8_t n0, n1, n2, n3;

  /* increment over the length of the string, three characters at a time */
  for (x = 0; x < dataLength; x += 3)
  {
    /* these three 8-bit (ASCII) characters become one 24-bit number */
    n = ((uint32_t)data[x]) << 16; //parenthesis needed, compiler depending on flags can do the shifting before conversion to uint32_t, resulting to 0

    if ((x + 1) < dataLength)
      n += ((uint32_t)data[x + 1]) << 8; //parenthesis needed, compiler depending on flags can do the shifting before conversion to uint32_t, resulting to 0

    if ((x + 2) < dataLength)
      n += data[x + 2];

    /* this 24-bit number gets separated into four 6-bit numbers */
    n0 = (uint8_t)(n >> 18) & 63;
    n1 = (uint8_t)(n >> 12) & 63;
    n2 = (uint8_t)(n >> 6) & 63;
    n3 = (uint8_t)n & 63;

    /*
       if we have one byte available, then its encoding is spread
       out over two characters
    */
    if (resultIndex >= resultSize) return 1;  /* indicate failure: buffer too small */
    result[resultIndex++] = base64chars[n0];
    if (resultIndex >= resultSize) return 1;  /* indicate failure: buffer too small */
    result[resultIndex++] = base64chars[n1];

    /*
       if we have only two bytes available, then their encoding is
       spread out over three chars
    */
    if ((x + 1) < dataLength)
    {
      if (resultIndex >= resultSize) return 1;  /* indicate failure: buffer too small */
      result[resultIndex++] = base64chars[n2];
    }

    /*
       if we have all three bytes available, then their encoding is spread
       out over four characters
    */
    if ((x + 2) < dataLength)
    {
      if (resultIndex >= resultSize) return 1;  /* indicate failure: buffer too small */
      result[resultIndex++] = base64chars[n3];
    }
  }

  /*
     create and add padding that is required if we did not have a multiple of 3
     number of characters available
  */
  if (padCount > 0)
  {
    for (; padCount < 3; padCount++)
    {
      if (resultIndex >= resultSize) return 1;  /* indicate failure: buffer too small */
      result[resultIndex++] = base64pad;
    }
  }
  //if(resultIndex >= resultSize) return 1;   /* indicate failure: buffer too small */
  //result[resultIndex] = 0; Do not null-terminate
  return 0;   /* indicate success */
}


//Payload requires 8 characters
//#define URL_BASE_LENGTH 9
#define URL_BASE_LENGTH 16
//static char url_buffer[17] = {0x03, 'r', 'u', 'u', '.', 'v', 'i', '/', '#'};
//static char url_buffer[30] = {0};
#define BASE_URL "https://ruu.vi/#"
//static uint8_t data_buffer[18] = { 0 };

/**
    Parses data into Ruuvi data format scale
    @param *data pointer to ruuvi_sensor_t object
    @param raw_t raw temperature as given by BME280, I.E 2-complement int32_t in celcius * 100, -2134 = 21.
  34
    @param raw_p raw pressure as given by BME280, uint32_t, multiplied by 256
    @param acceleration along 3 axes in milliG, X Y Z.
*/

void parseSensorData(struct ruuvi_sensor_t* data, int32_t raw_t, uint32_t raw_p, uint32_t raw_h, uint16_t vbat, int32_t acc[3])
{
  //    NRF_LOG_DEBUG("temperature: %d, pressure: %d, humidity: %d\r\n", raw_t, raw_p, raw_h);

  /*
    0:   uint8_t     format;          // (0x02 = realtime sensor readings base64)
    1:   uint8_t     humidity;        // one lsb is 0.5%
    2-3: uint16_t    temperature;     // Signed 8.8 fixed-point notation.
    4-5: uint16_t    pressure;        // (-50kPa)
  */
  //Convert raw values to ruu.vi specification
  //Round values: 1 deg C, 1 hPa, 1% RH
  data->format = 0x00; //Will be decided in encoding phase
  data->temperature = (raw_t < 0) ? 0x8000 : 0x0000; //Sign bit
  if (raw_t < 0) raw_t = 0 - raw_t; // disrecard sign
  data->temperature |= (((raw_t / 100) << 8));       //raw_t is 2:2 signed fixed point in base-10, Drop decimals, scale up to next byte.
  data->temperature |= (raw_t % 100);                //take decimals.
  data->pressure = (uint16_t)((raw_p >> 8) - 50000); //Scale into pa, Shift by -50000 pa as per Ruu.vi interface.
  data->humidity = (uint8_t)(raw_h >> 9);            //scale into 0.5%
  // Set accelerometer data
  data->accX = acc[0];
  data->accY = acc[1];
  data->accZ = acc[2];

  data->vbat = vbat;
}

/**
    Parses sensor values into RuuviTag format.
    @param char* data_buffer character array with length of 14 bytes
*/
void encodeToSensorDataFormat(uint8_t* data_buffer, struct ruuvi_sensor_t* data)
{
  //serialize values into a string
  data_buffer[0] = SENSOR_TAG_DATA_FORMAT;
  data_buffer[1] = data->humidity;
  data_buffer[2] = (data->temperature) >> 8;
  data_buffer[3] = (data->temperature) & 0xFF;
  data_buffer[4] = (data->pressure) >> 8;
  data_buffer[5] = (data->pressure) & 0xFF;
  data_buffer[6] = (data->accX) >> 8;
  data_buffer[7] = (data->accX) & 0xFF;
  data_buffer[8] = (data->accY) >> 8;
  data_buffer[9] = (data->accY) & 0xFF;
  data_buffer[10] = (data->accZ) >> 8;
  data_buffer[11] = (data->accZ) & 0xFF;
  data_buffer[12] = (data->vbat) >> 8;
  data_buffer[13] = (data->vbat) & 0xFF;
}

void encodeToUrlDataFromat(char* buf, ruuvi_sensor_t* data)
{
  //Create pseudo-unique name
  unsigned int mac0 = 100; // NRF_FICR->DEVICEID[0];
  uint8_t serial[2];
  serial[0] = mac0      & 0xFF;
  serial[1] = (mac0 >> 8) & 0xFF;

  //serialize values into a string
  char pack[8] = {0};
  pack[0] = WEATHER_STATION_URL_ID_FORMAT;
  uint8_t humidity = data->humidity / 4; //Round to 2 %
  pack[1] = humidity * 4;
  int16_t temperature = data->temperature;
  pack[2] = (temperature) >> 8;
  pack[3] = 0;          //Round off decimals
  uint16_t pressure =   data->pressure;
  pressure = pressure - (pressure % 100); //Round pressure to hPa accuracy
  pack[4] = (pressure) >> 8;
  pack[5] = (pressure) & 0xFF;
  pack[6] = serial[0];

  /// Encoding 48 bits using Base64 produces max 8 chars.
  memset(buf, 0, sizeof(URL_PAYLOAD_LENGTH));
  base64encode(pack, sizeof(pack), buf, URL_PAYLOAD_LENGTH);
}



