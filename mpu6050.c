
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75

#define MPU6050_ADDR 0xD0

void Single_WriteI2C(uint8_t REG_Address, uint8_t REG_data)
{
  uint8_t values[2] = {REG_Address,REG_data};
  ctx._error = nrf_drv_twi_tx(&ctx.m_twi_mma_7660, MPU6050_ADDR, values, 2, false);
}

void InitMPU6050()
{
#if 0
  Single_WriteI2C(PWR_MGMT_1, 0x00);
  Single_WriteI2C(SMPLRT_DIV, 0x07);
  Single_WriteI2C(CONFIG, 0x06);
  Single_WriteI2C(GYRO_CONFIG, 0x18);
  Single_WriteI2C(ACCEL_CONFIG, 0x01);
#else
  // sleep
  Single_WriteI2C(PWR_MGMT_1, 0x40);
#endif
}
