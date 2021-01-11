#ifndef WARP_BUILD_ENABLE_DEVINA219
#define WARP_BUILD_ENABLE_DEVINA219
#endif

i2c_status_t initINA219(i2c_device_t slave, uint16_t menuI2cPullupValue);
i2c_status_t setINA219Calibration(i2c_device_t slave, uint16_t calibration_value, uint16_t menuI2cPullupValue);
i2c_status_t readRegisterINA219(i2c_device_t slave, uint8_t device_register, uint8_t * i2c_buffer, uint16_t menuI2cPullupValue);
i2c_status_t printRegisterINA219(i2c_device_t slave, uint8_t device_register, uint16_t menuI2cPullupValue);
uint32_t readCurrentINA219(i2c_device_t slave, uint16_t current_LSB, uint16_t menuI2cPullupValue);
