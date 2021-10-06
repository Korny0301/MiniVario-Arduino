# MiniVario-Arduino

### Forked by IvkoPivko/MiniVario-Arduino
Forked to include BMP280 sensor and clean up source code. Thanks to IvkoPivko for this great code basis!


Acoustic mini variometer with Bluetooth function optimized for use as paraglider. This variometer provides:
- acoustic representation of current rise/sink
- bluetooth with several supported protocols (can be disabled)
- monitoring of battery capacity

### Arduino Libraries

Barometric Pressure & Temperature Sensor Arduino Library (install only the required one and activate in source code):
- https://github.com/jarzebski/Arduino-MS5611
- https://github.com/adafruit/Adafruit_BMP280_Library

Optional low power mode:
- https://github.com/rocketscream/Low-Power  

![Wiring](https://raw.githubusercontent.com/IvkoPivko/MiniVario-Arduino/master/Wiring/MiniPro_3-3V_Vario_Wiring_BT_Bat.png)

## Details

### Bluetooth module

My bluetooth module is an HM-10 assembled on another PCB for easy installation due to 'normal' pin sizing, no SMD assembling required. Depending on the used bluetooth module you might need to reduce the voltage on the input pins of the bluetooth module. For use with HM-10, HC-05 or HC-06 bluetooth module in combination with 5V transmission lines, I must decrease the voltage on the bluetooth RX line since it just supports 3.3V corresponding to print on PCB.

To reduce the voltage of RX (Arduino TX) from 5V to 3.3V you can use a voltage divider through two resistors:

U -> R_1 -> R_2 -> GND (with U_2 above R_2)

U_2 = U / (R_1 + R_2) * R_2

3.3V = 5V / (R_1 + R_2) * R_2

Since I have lots of those resistors, I used for R2 = 9.1kOhm and R1 = 5kOhm (calculated R1 = 4.687kOhm). Bluetooth module RX is connected to voltage above R_2 (between both resistors).

Notice, that the HC-05 (and 06) are not compatible with apple products as iPhone/iPad. Those device will not find the bluetooth module due to missing licenses (google it if you are interested). Even my MacbookPro was not able to connect - but at least it found the device while scanning. Therefore I used the HM-10 that looks the same from a software side of view. 


### Feature requests / nice to have / maybe implemented in future

- calibration of height, maybe with GPS of smartphone

