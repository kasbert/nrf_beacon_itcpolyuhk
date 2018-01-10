
#include <nrf_delay.h>
#include "ruuvi.h"

#include "bmp180.c"
#include "ap3216.c"
#include "mpu6050.c"

void init_sensors() {
  bmp180_begin();
  //alsStart();
  ////psStart();
  alsPsStop();
  InitMPU6050();
}  

void handle_sensors(uint8_t *buffer) {
  static int32_t  raw_t  = 0*100;
  static uint32_t raw_p = 900 * 10000;
  static uint32_t raw_h = 1 * 1000;
  static int32_t acc[3] = {1, 2, 3};
  static uint16_t vbat = 100;

  ruuvi_sensor_t data;

  //setColor(COLOR_GREEN);
  double T;
  char status = bmp180_startTemperature();
  if (status != 0)
    {
      // Wait for the measurement to complete:
      nrf_delay_ms(status);
      status = bmp180_getTemperature(&T);
      if (status != 0) {
	#if BLE_SERIAL
	bleSerial.println("temp:");
	bleSerial.println(T);
	#endif
	raw_t = T * 100;
	status = bmp180_startPressure(0);
	if (status != 0)
	  {
	    double P;
	    // Wait for the measurement to complete:
	    nrf_delay_ms(status);
	    status = bmp180_getPressure(&P, &T);
	    if (status != 0)
	      {
		raw_p = P * 25600;
	      }
	  }
      }
    }
  
#if 0
  // Ambient light as humidity
  uint16_t als = getALS();
  raw_h = als * 100 / 65;
#endif
  
  parseSensorData(&data, raw_t, raw_p, raw_h, vbat, acc);
  encodeToUrlDataFromat((char *)buffer, &data);

  //bluetooth_advertise_data(data_buffer, sizeof(data_buffer));
  //eddystone_advertise_url(url_buffer, sizeof(url_buffer));
  //  eddystoneBeacon.setURI(url_buffer);
  //delay(10);
  //setColor(COLOR_OFF);
}
