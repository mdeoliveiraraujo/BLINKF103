# BLINKF103
 
That's the same as the _Angle_Measure_ repo without the MPU6050 implementation.

Shoutout to those who were here before me:

https://controllerstech.com/ (SD Card (SPI))
https://github.com/mbilsky, https://github.com/Flydroid and https://github.com/dariosalvi78 (ADS1256 (SPI)).

Note that this one here has a _trashy_ implementation using ASCII to record data directly into the SD Card. I used _printf_ to convert bytes into text and lost a generous amount of time there. _Angle_Measure_ implementation (bytes directly into SD Card) showed itself much faster.

Enjoy!