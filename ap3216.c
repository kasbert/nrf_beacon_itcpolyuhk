// Interfacing the AP3216 light / proximity sensor with Arduino UNO

// By RoboRemo
// www.roboremo.com

// Big thanks to ICStation for providing the AP3216 sensor
// http://www.icstation.com/ap3216-ambient-light-sensorals-proximity-sensorps-p-7958.html

// Command examples:
// "write 0x00 0x01\n" - will write value 0x01 to the register 0x00
// "read 0x0C\n" - will read the value from register 0x0C
// "als start\n" - will start streaming the value from the ALS (ambient light sensor)
// "ps start\n" - will start streaming the value from the PS (proximity sensor)
// "stop\n" - will stop streaming the ALS / PS data.

// Commands can be sent using Serial Monitor / Terminal,
// Or using the RoboRemo app from Google Play.

// RoboRemo app can also display a nice plot of the ALS / PS data,
// and also log to a file on the sdcard of the phone.


// Hardware wiring:
// Arduino     AP3216
//             VLED --,
// GND ------- GND   |R| 240 Ohm
// 3.3V ------ VCC ---'
// A5 -------- SCL
// A4 -------- SDA

#define AP3216_ADDR 0x1E

bool als_on = false;
bool ps_on = false;

void AP3216_write(uint8_t regAddress, uint8_t value);
int AP3216_read(uint8_t regAddress);

void alsStart() {
  AP3216_write(0x00, 0x01);
  als_on = true;
}

void alsPsStop() {
  als_on = false;
  ps_on = false;
  AP3216_write(0x00, 0x00);
}

void psStart() {
  AP3216_write(0x00, 0x02);
  ps_on = true;
}

uint16_t getALS() {
  uint8_t a = AP3216_read(0x0D); // ALS Data HIGH Byte
  uint8_t b = AP3216_read(0x0C); // ALS Data LOW Byte
  return (a << 8) | b;
}

uint16_t getPS() {
    int a = AP3216_read(0x0F) & 0b00111111; // PS Data HIGH 6 bits
    int b = AP3216_read(0x0E) & 0b00001111; // PS Data LOW 4 bits
    return (a << 4) | b;
}

void AP3216_write(uint8_t regAddress, uint8_t value) {
  uint8_t values[2] = {regAddress,value};
  ctx._error = nrf_drv_twi_tx(&ctx.m_twi_mma_7660, AP3216_ADDR, values, 2, false);
}

int AP3216_read(uint8_t regAddress) {
  uint8_t value = regAddress;
  ret_code_t _error;
#if USE_CALLBACK
  ctx.m_xfer_done = false;
  ctx.return_data = &value;
  ctx.return_data_len = 1;
   _error = nrf_drv_twi_tx(&ctx.m_twi_mma_7660, AP3216_ADDR, &value, 1, false);
  while (!ctx.m_xfer_done);
#else
  _error = nrf_drv_twi_tx(&ctx.m_twi_mma_7660, AP3216_ADDR, &value, 1, false);
  _error = nrf_drv_twi_rx(&ctx.m_twi_mma_7660, AP3216_ADDR, &value, 1);
  APP_ERROR_CHECK(_error);
#endif
  return value;
}


