Instructions for getting this working.
This was build using CLion, hence the .idea files.

Uses a NANO 328P, with hardware SPI (pins 11 & 13).
To wire up the board:

```
Arduino Pin ----> LCD
13                CL
11                SI
4                 DC
5                 OC
6                 R
5V                +
GND               G
```

For the Accelerometer, we need I2C.
For the Nano, the 328P uses the same wiring as the Uno,
so A4 is SDA, A5 is SCL.

The MPU is 5V, GND, SCL, SDA, XDA, XCL, ADO, INT
Can be wired as:

```
Arduino Pin ----> MPU
5V                 5V
GND                GND
A4                 SDA
A5                 SCL
```
