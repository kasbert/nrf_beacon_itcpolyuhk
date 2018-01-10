
#include "nrf_drv_twi.h"
#include <math.h>
#include "app_util_platform.h"

#define PIN_WIRE_SDA 9
#define PIN_WIRE_SCL 10
  
#define BMP180_ADDR 0x77 // 7-bit address

#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

#define readInt(a,v) bmp180_readInt(a,&v)
#define readUInt(a,v) bmp180_readUInt(a,&v)
char bmp180_readInt(char address, int16_t *value);
char bmp180_readUInt(char address, uint16_t *value);
char bmp180_readBytes(unsigned char *values, char length);

static struct bmp180 {
  int16_t AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
  uint16_t AC4,AC5,AC6;
  double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;
  char _error;

  /* Indicates if reading operation from accelerometer has ended. */
  volatile bool m_xfer_done ;
  /* Indicates if setting mode operation has ended. */
  //volatile bool m_set_mode_done;
  nrf_drv_twi_t m_twi_mma_7660;
  volatile uint8_t *return_data;
  volatile int return_data_len;
} ctx;


#define USE_CALLBACK 0

#if USE_CALLBACK
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
  ret_code_t err_code;
  LEDS_ON(BSP_LED_2_MASK);
  switch(p_event->type)
    {
    case NRF_DRV_TWI_EVT_DONE:
      if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
	{
	  //if(ctx.m_set_mode_done != true) {
	  //  ctx.m_set_mode_done  = true;
	  //  return;
	  //}
	  ctx.m_xfer_done = false;
	  if (ctx.return_data) {
	    err_code = nrf_drv_twi_rx(&ctx.m_twi_mma_7660, BMP180_ADDR, (uint8_t*)ctx.return_data, ctx.return_data_len);
	    ctx.return_data = 0;
	    APP_ERROR_CHECK(err_code);
	  }
	}
      else
	{
	  //read_data(return_data);
	  ctx.m_xfer_done = true;
	}
      break;
    default:
      break;
    }
  LEDS_OFF(BSP_LED_1_MASK);
}
#else
#define twi_handler 0
#endif

char bmp180_readBytes(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
  //Wire.beginTransmission(BMP180_ADDR);
  //Wire.write(values[0]);
  //_error = Wire.endTransmission();
  ret_code_t _error;
#if USE_CALLBACK
  ctx.m_xfer_done = false;
  ctx.return_data = values;
  ctx.return_data_len = length;
  _error = nrf_drv_twi_tx(&ctx.m_twi_mma_7660, BMP180_ADDR, values, 1, false);
  while (!ctx.m_xfer_done);
#else
  _error = nrf_drv_twi_tx(&ctx.m_twi_mma_7660, BMP180_ADDR, values, 1, false);
  _error = nrf_drv_twi_rx(&ctx.m_twi_mma_7660, BMP180_ADDR, values, length);
  APP_ERROR_CHECK(_error);
#endif
  return !_error;
}


char bmp180_writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
  ret_code_t _error = nrf_drv_twi_tx(&ctx.m_twi_mma_7660, BMP180_ADDR, values, length, false);
  //Wire.beginTransmission(BMP180_ADDR);
  //Wire.write(values,length);
  //_error = Wire.endTransmission();
  if (_error == 0)
    return(1);
  else
    return(0);
}

void twi_init (void)

{
  const nrf_drv_twi_config_t twi_mma_7660_config = {
    .scl                = PIN_WIRE_SCL,
    .sda                = PIN_WIRE_SDA,
    .frequency          = NRF_TWI_FREQ_100K,
   .interrupt_priority = APP_IRQ_PRIORITY_HIGH
  };

  ctx.m_xfer_done = true;
  //ctx.m_set_mode_done = false;
  nrf_drv_twi_t m_twi_mma_7660 = NRF_DRV_TWI_INSTANCE(0);
  ctx.m_twi_mma_7660 = m_twi_mma_7660; //NRF_DRV_TWI_INSTANCE(0);

  ret_code_t err_code;
  err_code = nrf_drv_twi_init(&ctx.m_twi_mma_7660, &twi_mma_7660_config, twi_handler, &ctx);

  APP_ERROR_CHECK(err_code);
  nrf_drv_twi_enable(&ctx.m_twi_mma_7660);
}


char bmp180_begin()
// Initialize library for subsequent pressure measurements
{
	double c3,c4,b1;
	
	// Start up the Arduino's "wire" (I2C) library:
	twi_init();
	//Wire.begin();

	// The BMP180 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.

	// Retrieve calibration data from device:
	
	if (readInt(0xAA,ctx.AC1) &&
		readInt(0xAC,ctx.AC2) &&
		readInt(0xAE,ctx.AC3) &&
		readUInt(0xB0,ctx.AC4) &&
		readUInt(0xB2,ctx.AC5) &&
		readUInt(0xB4,ctx.AC6) &&
		readInt(0xB6,ctx.VB1) &&
		readInt(0xB8,ctx.VB2) &&
		readInt(0xBA,ctx.MB) &&
		readInt(0xBC,ctx.MC) &&
		readInt(0xBE,ctx.MD))
	{

		// All reads completed successfully!

		// If you need to check your math using known numbers,
		// you can uncomment one of these examples.
		// (The correct results are commented in the below functions.)

		// Example from Bosch datasheet
		// AC1 = 408; AC2 = -72; AC3 = -14383; AC4 = 32741; AC5 = 32757; AC6 = 23153;
		// B1 = 6190; B2 = 4; MB = -32768; MC = -8711; MD = 2868;

		// Example from http://wmrx00.sourceforge.net/Arduino/BMP180-Calcs.pdf
		// AC1 = 7911; AC2 = -934; AC3 = -14306; AC4 = 31567; AC5 = 25671; AC6 = 18974;
		// VB1 = 5498; VB2 = 46; MB = -32768; MC = -11075; MD = 2432;

		/*
		Serial.print("AC1: "); Serial.println(AC1);
		Serial.print("AC2: "); Serial.println(AC2);
		Serial.print("AC3: "); Serial.println(AC3);
		Serial.print("AC4: "); Serial.println(AC4);
		Serial.print("AC5: "); Serial.println(AC5);
		Serial.print("AC6: "); Serial.println(AC6);
		Serial.print("VB1: "); Serial.println(VB1);
		Serial.print("VB2: "); Serial.println(VB2);
		Serial.print("MB: "); Serial.println(MB);
		Serial.print("MC: "); Serial.println(MC);
		Serial.print("MD: "); Serial.println(MD);
		*/
		
		// Compute floating-point polynominals:

		c3 = 160.0 * pow(2,-15) * ctx.AC3;
		c4 = pow(10,-3) * pow(2,-15) * ctx.AC4;
		b1 = pow(160,2) * pow(2,-30) * ctx.VB1;
		ctx.c5 = (pow(2,-15) / 160) * ctx.AC5;
		ctx.c6 = ctx.AC6;
		ctx.mc = (pow(2,11) / pow(160,2)) * ctx.MC;
		ctx.md = ctx.MD / 160.0;
		ctx.x0 = ctx.AC1;
		ctx.x1 = 160.0 * pow(2,-13) * ctx.AC2;
		ctx.x2 = pow(160,2) * pow(2,-25) * ctx.VB2;
		ctx.y0 = c4 * pow(2,15);
		ctx.y1 = c4 * c3;
		ctx.y2 = c4 * b1;
		ctx.p0 = (3791.0 - 8.0) / 1600.0;
		ctx.p1 = 1.0 - 7357.0 * pow(2,-20);
		ctx.p2 = 3038.0 * 100.0 * pow(2,-36);
		// Success!
		return(1);
	}
	else
	{
		// Error reading calibration data; bad component or connection?
		return(0);
	}
}


char bmp180_readInt(char address, int16_t *value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (bmp180_readBytes(data,2))
	{
		*value = (int16_t)((data[0]<<8)|data[1]);
		//if (*value & 0x8000) *value |= 0xFFFF0000; // sign extend if negative
		return(1);
	}
	value = 0;
	return(0);
}


char bmp180_readUInt(char address, uint16_t *value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (bmp180_readBytes(data,2))
	{
		*value = (((uint16_t)data[0]<<8)|(uint16_t)data[1]);
		return(1);
	}
	value = 0;
	return(0);
}


char bmp180_startTemperature(void)
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
{
	unsigned char data[2], result;
	
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	result = bmp180_writeBytes(data, 2);
	if (result) // good write?
		return(5); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}


char bmp180_getTemperature(double *T)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
	unsigned char data[2];
	char result;
	double tu, a;
	
	data[0] = BMP180_REG_RESULT;

	result = bmp180_readBytes(data, 2);
	if (result) // good read, calculate temperature
	{
		tu = (data[0] * 256.0) + data[1];

		//example from Bosch datasheet
		//tu = 27898;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
		//tu = 0x69EC;
		
		a = ctx.c5 * (tu - ctx.c6);
		*T = a + (ctx.mc / (a + ctx.md));

		/*		
		Serial.println();
		Serial.print("tu: "); Serial.println(tu);
		Serial.print("a: "); Serial.println(a);
		Serial.print("T: "); Serial.println(*T);
		*/
	}
	return(result);
}


char bmp180_startPressure(char oversampling)
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
	unsigned char data[2], result, delay;
	
	data[0] = BMP180_REG_CONTROL;

	switch (oversampling)
	{
		case 0:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
		case 1:
			data[1] = BMP180_COMMAND_PRESSURE1;
			delay = 8;
		break;
		case 2:
			data[1] = BMP180_COMMAND_PRESSURE2;
			delay = 14;
		break;
		case 3:
			data[1] = BMP180_COMMAND_PRESSURE3;
			delay = 26;
		break;
		default:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
	}
	result = bmp180_writeBytes(data, 2);
	if (result) // good write?
		return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}


char bmp180_getPressure(double *P, double *T)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
	unsigned char data[3];
	char result;
	double pu,s,x,y,z;
	
	data[0] = BMP180_REG_RESULT;

	result = bmp180_readBytes(data, 3);
	if (result) // good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

		//example from Bosch datasheet
		//pu = 23843;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;	
		//pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);
		
		s = *T - 25.0;
		x = (ctx.x2 * pow(s,2)) + (ctx.x1 * s) + ctx.x0;
		y = (ctx.y2 * pow(s,2)) + (ctx.y1 * s) + ctx.y0;
		z = (pu - x) / y;
		*P = (ctx.p2 * pow(z,2)) + (ctx.p1 * z) + ctx.p0;

		/*
		Serial.println();
		Serial.print("pu: "); Serial.println(pu);
		Serial.print("T: "); Serial.println(*T);
		Serial.print("s: "); Serial.println(s);
		Serial.print("x: "); Serial.println(x);
		Serial.print("y: "); Serial.println(y);
		Serial.print("z: "); Serial.println(z);
		Serial.print("P: "); Serial.println(*P);
		*/
	}
	return(result);
}


double bmp180_sealevel(double P, double A)
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}


double bmp180_altitude(double P, double P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}


char bmp180_getError(void)
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error
{
	return(ctx._error);
}
