/*
	Authored 2016-2018. Phillip Stanley-Marbell, Youchao Wang, James Meech.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceBME680State;
extern volatile uint8_t			deviceBME680CalibrationValues[];
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


void
initBME680(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskPressure | kWarpTypeMaskTemperature);

	return;
}

WarpStatus
writeSensorRegisterBME680(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	if (deviceRegister > 0xFF)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterBME680(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);

	/*
	 *	We only check to see if it is past the config registers.
	 *
	 *	TODO: We should eventually numerate all the valid register addresses
	 *	(configuration, control, and calibration) here.
	 */
	if (deviceRegister > kWarpSensorConfigurationRegisterBME680CalibrationRegion2End)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBME680State.i2cBuffer,
							1,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


WarpStatus
configureSensorBME680(uint8_t payloadCtrl_Hum, uint8_t payloadCtrl_Meas, uint8_t payloadGas_0, uint16_t menuI2cPullupValue)
{
	uint8_t		reg, index = 0;
	WarpStatus	status1, status2, status3, status4 = 0;

	status1 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Hum,
							payloadCtrl_Hum,
							menuI2cPullupValue);

	status2 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas,
							payloadCtrl_Meas,
							menuI2cPullupValue);

	status3 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Gas_0,
							payloadGas_0,
							menuI2cPullupValue);

	/*
	 *	Read the calibration registers
	 */
	for (	reg = kWarpSensorConfigurationRegisterBME680CalibrationRegion1Start;
		reg < kWarpSensorConfigurationRegisterBME680CalibrationRegion1End;
		reg++)
	{
		status4 |= readSensorRegisterBME680(reg, 1 /* numberOfBytes */);
		deviceBME680CalibrationValues[index++] = deviceBME680State.i2cBuffer[0];
	}

	for (	reg = kWarpSensorConfigurationRegisterBME680CalibrationRegion2Start;
		reg < kWarpSensorConfigurationRegisterBME680CalibrationRegion2End;
		reg++)
	{
		status4 |= readSensorRegisterBME680(reg, 1 /* numberOfBytes */);
		deviceBME680CalibrationValues[index++] = deviceBME680State.i2cBuffer[0];
	}

	return (status1 | status2 | status3 | status4);
}


void 
newSensorDataBME680(uint8_t* new_temp, uint8_t* new_hum, uint8_t* new_press, uint16_t* new_gas_res, uint16_t menuI2cPullupValue)
{       
       	uint16_t        readSensorRegisterValueLSB;
        uint16_t        readSensorRegisterValueMSB;
        uint16_t        readSensorRegisterValueXLSB;
        uint32_t        unsignedRawAdcValue;
        WarpStatus      triggerStatus, i2cReadStatusMSB, i2cReadStatusLSB, i2cReadStatusXLSB;

        triggerStatus = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas,
                                                        0b00100101,
                                                        menuI2cPullupValue);
	/*
         *      First, trigger a measurement of temperature
         */
        i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_msb, 1);
        readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
        i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_lsb, 1);
        readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
        i2cReadStatusXLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_xlsb, 1);
        readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];

        unsignedRawAdcValue =
                        ((readSensorRegisterValueMSB & 0xFF)  << 12) |
                        ((readSensorRegisterValueLSB & 0xFF)  << 4)  |
                        ((readSensorRegisterValueXLSB & 0xF0) >> 4);

        uint16_t par_t1 = (deviceBME680CalibrationValues[34] << 8) | deviceBME680CalibrationValues[33];
        int16_t par_t2 = (deviceBME680CalibrationValues[2] << 8) | deviceBME680CalibrationValues[1];
        int8_t par_t3 = deviceBME680CalibrationValues[3];
        int32_t var1_t;
        int32_t var2_t;
        int32_t var3_t;
        int32_t t_fine;
        int16_t temp_comp;

        var1_t = (unsignedRawAdcValue >> 3) - (par_t1 << 1);
        var2_t = (var1_t * par_t2) >> 11;
        var3_t = ((((var1_t >> 1) * (var1_t >> 1)) >> 12) * (par_t3 << 4)) >> 14;
        t_fine = var2_t + var3_t;
        temp_comp = (((t_fine) * 5) + 128) >> 8;
	temp_comp = temp_comp / 100;

	if ((triggerStatus != kWarpStatusOK) || (i2cReadStatusMSB != kWarpStatusOK) || (i2cReadStatusLSB != kWarpStatusOK) || (i2cReadStatusXLSB != kWarpStatusOK))
        {
                *new_temp = 0;
        }
        else
        {
   		*new_temp = (uint8_t) (temp_comp & 0x000000FF);
        }
	
	/*
         *      Then, trigger a measurement of pressure
         */

        i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_msb, 1);
        readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
        i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_lsb, 1);
        readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
        i2cReadStatusXLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_xlsb, 1);
        readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];

        unsignedRawAdcValue =
                        ((readSensorRegisterValueMSB & 0xFF)  << 12) |
                        ((readSensorRegisterValueLSB & 0xFF)  << 4)  |
                        ((readSensorRegisterValueXLSB & 0xF0) >> 4);

        uint16_t par_p1 = (deviceBME680CalibrationValues[6] << 8) | deviceBME680CalibrationValues[5];
        int16_t par_p2 = (deviceBME680CalibrationValues[8] << 8) | deviceBME680CalibrationValues[7];
        int8_t par_p3 = deviceBME680CalibrationValues[9];
        int16_t par_p4 = (deviceBME680CalibrationValues[12] << 8) | deviceBME680CalibrationValues[11];
        int16_t par_p5 = (deviceBME680CalibrationValues[14] << 8) | deviceBME680CalibrationValues[13];
        int8_t par_p6 = deviceBME680CalibrationValues[16];
        int8_t par_p7 = deviceBME680CalibrationValues[15];
        int16_t par_p8 = (deviceBME680CalibrationValues[20] << 8) | deviceBME680CalibrationValues[19];
        int16_t par_p9 = (deviceBME680CalibrationValues[22] << 8) | deviceBME680CalibrationValues[21];
        uint8_t par_p10 = deviceBME680CalibrationValues[23];
       	int32_t var1_p;
        int32_t var2_p;
        int32_t var3_p;
       	int32_t press_comp;
        
        var1_p = (t_fine >> 1) - 64000;
        var2_p = ((((var1_p >> 2) * (var1_p >> 2)) >> 11) * par_p6) >> 2;
        var2_p = var2_p + ((var1_p * par_p5) << 1);
        var2_p = (var2_p >> 2) + (par_p4 << 16);
        var1_p = (((((var1_p >> 2) * (var1_p >> 2)) >> 13) * (par_p3 << 5)) >> 3) + ((par_p2 * var1_p) >> 1);
        var1_p = var1_p >> 18;
        var1_p = ((32768 + var1_p) * par_p1) >> 15;
        press_comp = 1048576 - unsignedRawAdcValue;
        press_comp = ((press_comp - (var2_p >> 12)) * 3125);
	
	if (press_comp >= (1 << 30))
 	{
		press_comp = ((press_comp / var1_p) << 1);
 	}
	else
	{
		press_comp = ((press_comp << 1) / var1_p);
	}
	var1_p = (par_p9 * (((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
	var2_p = ((press_comp >> 2) * par_p8) >> 13;
	var3_p = ((press_comp >> 8) * (press_comp >> 8) * (press_comp >> 8) * par_p10) >> 17;
	press_comp = (press_comp) + ((var1_p + var2_p + var3_p + (par_p7 << 7)) >> 4);
	press_comp = press_comp / 100;

        if ((triggerStatus != kWarpStatusOK) || (i2cReadStatusMSB != kWarpStatusOK) || (i2cReadStatusLSB != kWarpStatusOK) || (i2cReadStatusXLSB != kWarpStatusOK))
        {
                *new_press = 0;
        }
	
        else
        {
                *new_press = (uint8_t) (press_comp & 0x000000FF);
        }

	/*
         *      Next, trigger a measurement of humidity
         */
        i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680hum_msb, 1);
        readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
        i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680hum_lsb, 1);
        readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
        unsignedRawAdcValue = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        uint16_t par_h1 = (deviceBME680CalibrationValues[27] << 4) | ((deviceBME680CalibrationValues[26] & 0xF0) >>4);
        uint16_t par_h2 = (deviceBME680CalibrationValues[25] << 4) | ((deviceBME680CalibrationValues[26] & 0xF0) >>4);
        int8_t par_h3 = deviceBME680CalibrationValues[28];
        int8_t par_h4 = deviceBME680CalibrationValues[29];
        int8_t par_h5 = deviceBME680CalibrationValues[30];
        uint8_t par_h6 = deviceBME680CalibrationValues[31];
        int8_t par_h7 = deviceBME680CalibrationValues[32];
	int32_t var1;
        int32_t var2;
        int32_t var3;
        int32_t var4;
        int32_t var5;
        int32_t var6;
        int32_t hum_comp;

        var1 = unsignedRawAdcValue - (par_h1 << 4) - (((temp_comp * par_h3) / 100) >> 1);
        var2 = (par_h2 * (((temp_comp * par_h4) / 100) + (((temp_comp * ((temp_comp * par_h5) / ( 100))) >> 6) / 100) + (1 << 14))) >> 10;
        var3 = var1 * var2;
        var4 = ((par_h6 << 7) + ((temp_comp * par_h7) / 100)) >> 4;
        var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
        var6 = (var4 * var5) >> 1;
        hum_comp = (((var3 + var6) >> 10) * (1000)) >> 12;

        if (hum_comp > 100000)
        {
                hum_comp = 100000;
        }

        else if (hum_comp < 0)
        {
                hum_comp = 0;
        }

	hum_comp = hum_comp / 1000;

	if ((triggerStatus != kWarpStatusOK) || (i2cReadStatusMSB != kWarpStatusOK) || (i2cReadStatusLSB != kWarpStatusOK))
        {
                *new_hum = 0;
        }
        else
        {
        	*new_hum = (uint8_t) (hum_comp & 0x000000FF);
        }

	/*
         *      Lastly, trigger a measurement of gas resistance
         */
        int8_t res_heat_val;
        int8_t res_heat_range;
        int64_t range_switching_error;
        int32_t par_g1;
        int16_t par_g2;
        int32_t par_g3;
        uint8_t res_heat_x;
        int32_t res_heat_x100;
        int32_t target_temp;
	int32_t var1_g;
        int32_t var2_g;
        int32_t var3_g;
        int32_t var4_g;
        int32_t var5_g;

        readSensorRegisterBME680(0x00, 1);
        res_heat_val = deviceBME680State.i2cBuffer[0];
        readSensorRegisterBME680(0x02, 1);
        res_heat_range = (deviceBME680State.i2cBuffer[0] >> 4) & 0x03;
        readSensorRegisterBME680(0x04, 1);
        range_switching_error = deviceBME680State.i2cBuffer[0];

        par_g1 = deviceBME680CalibrationValues[37];
        par_g2 = (deviceBME680CalibrationValues[36] << 8) | deviceBME680CalibrationValues[35];
        par_g3 = deviceBME680CalibrationValues[38];
        target_temp = 320;

        var1_g = ((25 * par_g3) / 10) << 8;
        var2_g = (par_g1 + 784) * (((((par_g2 + 154009) * target_temp * 5) / 100) + 3276800) / 10);
        var3_g = var1_g + (var2_g >> 1);
        var4_g = (var3_g / (res_heat_range + 4));
        var5_g = (131 * res_heat_val) + 65536;
        res_heat_x100 = ((var4_g / var5_g) - 250) * 34;
        res_heat_x = (res_heat_x100 + 50) / 100;

        writeSensorRegisterBME680(0x5A, res_heat_x, menuI2cPullupValue);
        writeSensorRegisterBME680(0x64, 0x59, menuI2cPullupValue);
        writeSensorRegisterBME680(0x71, 0b00010000, menuI2cPullupValue);

        /*
         *      Check for new data
         */
        int8_t new_data = 0;
        int8_t gas_res_range = 0;
        int8_t gas_res_MSB;
        int8_t gas_res_LSB;
        int16_t gas_res_adc = 0;

        while (new_data == 0)
        {
                readSensorRegisterBME680(0x1D, 1);
                new_data = deviceBME680State.i2cBuffer[0] >> 7;
        }

        readSensorRegisterBME680(0x2A, 1);
        gas_res_MSB = deviceBME680State.i2cBuffer[0];
        readSensorRegisterBME680(0x2B, 1);
        gas_res_LSB = deviceBME680State.i2cBuffer[0];

	if (((gas_res_LSB & 0x30) >> 4)  == 0x03)
        {
                gas_res_adc = (gas_res_MSB << 2) | (gas_res_LSB >> 6);
                gas_res_range = gas_res_LSB & 0x0F;
        }

        int64_t var1_comp;
        uint64_t var2_comp;
        int64_t var3_comp;
        uint32_t gas_res_comp;

        /**Look up table 1 for the possible gas range values */
        uint32_t lookupTable1[16] = { UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
                UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
                UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
                UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647) };

        /**Look up table 2 for the possible gas range values */
        uint32_t lookupTable2[16] = { UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
                UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016),
                UINT32_C(8000000), UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000),
                UINT32_C(250000), UINT32_C(125000) };

        var1_comp = (int64_t) ((1340 + (5 * range_switching_error)) * ((int64_t) lookupTable1[gas_res_range])) >> 16;
        var2_comp = (((int64_t) ((int64_t) gas_res_adc << 15) - (int64_t) (16777216)) + var1_comp);
        var3_comp = (((int64_t) lookupTable2[gas_res_range] * (int64_t) var1_comp) >> 9);
        gas_res_comp = (uint32_t) ((var3_comp + ((int64_t) var2_comp >> 1)) / (int64_t) var2_comp);


        if ((triggerStatus != kWarpStatusOK) || (i2cReadStatusMSB != kWarpStatusOK) || (i2cReadStatusLSB != kWarpStatusOK))
        {
                *new_gas_res = 0;
        }
        else
        {
                *new_gas_res = gas_res_comp;
        }
}
