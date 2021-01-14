# Coursework 5 - Monitoring Indoor Air Quality
### Rae Zhao (rz295)
### Peterhouse

## Project Overview
The project aims to implement an Indoor Air Quality monitor on the FRDMKL03 development board using Warp firmware. The IAQ monitor is able to display temperature in degree Celsius, humidity in %rH, and IAQ index (between a range from 0-500) on SSD1331 OLED display with readings obtained from the BME680 environmental sensor. In addition, the system is able to determine the quality of air based on threshold limits set for temperature, humidity, IAQ index and indicate good air quality with a smiley face or a warning sign otherwise.

For example, the thresholds set for this monitor is such that it will display a green smiley face if:
1. Temperature is between 20 degC and 25 degC
2. Humidity is between 40%rH and 60%rH
3. IAQ index is less than 50

A yellow smiley face if:
1. Temperature is between 20 degC and 25 degC
2. Humidity is between 40%rH and 60%rH
3. IAQ index is between 50 and 100

And a warning sign otherwise.

IAQ index in this case is calculated based on humidity and gas resistance readings obtained from BME680 instead of using BSEC software to obtain IAQ index as one would want to directly output the IAQ index readings onto OLED display. Humidity contributes 25% to the final IAQ score and gas resistance contributes 75%.

An image of the final board layout is as shown below:
![Final Board Layout](layout.png)

## Repository Layout
The directory `src/boot/ksdk1.1.0` in this repository includes the key files that were modified for the project, namely as follows:
```
CMakeLists.txt
devBME680.c
devBME680.h
devSSD1331.c
devSSD1331.h
warp-kl03-ksdk1.1-boot.c (mainly in lines 1229-1309 before the while (1) loop)
warp.h
```
In `devBME680.c`, raw readings `unsignedRawAdcValue` of temperature, humidity and gas resistance are converted to calibrated readings using the conversion routines from [BME680 Driver](https://github.com/BoschSensortec/BME680_driver) in Bosch Sensortec Libray.

In `devSSD1331.c` includes functions that can print integers, strings of characters, and smiley faces, which are adapted from [SSD1331 OLED Driver Library](https://os.mbed.com/users/star297/code/ssd1331//file/4385fd242db0/ssd1331.cpp/). In addtion, `devSSD1331init` function includes printing temperature, humidity, IAQ index, smiley faces and warning sign onto OLED display.

In `warp-kl03-ksdk1.1-boot.c` (lines 1229-1309), the sensor BME680 is first being configured. Then, compensated values for temperature, humidity and gas resistance are calculated and called using function `newSensorDataBME680` in `devBME680.c`. Next, IAQ index is calculated in the code using 75% of gas resistance and 25% of humidity. Finally, the readings are displayed onto OLED display using the function `devSSD1331init` in file `devSSD1331.c`. Some redundant code in `warp-kl03-ksdk1.1-boot.c` related to unused sensors has been removed to free up memory space of the device.

