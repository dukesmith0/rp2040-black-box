# rp2040-black-box

# Additional Boards Manager URL:
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

install Raspberry Pi Pico/RP2040/RP2350 by Earle F Philhower, III in board manager

Navigate to the Tools > Boards > Raspberry Pi Pico RP2040/RP2350 > Adafruit Feather RP2040 Adalogger

Note: If there is no serial Port available in the dropdown, or an invalid one appears - don't worry about it! The RP2040 does not actually use a serial port to upload, so its OK if it does not appear if in manual bootload mode. You will see a serial port appear after uploading your first sketch.

# Necessary Libraries (include all dependencies)
Adafruit_BMP3XX  
Adafruit GPS Library  
RTClib  
Adafruit LIS3MDL Library  
Adafruit LSM6DS  

# Individual Board Guides:
Adafruit Feather RP2040 Adalogger - https://learn.adafruit.com/adafruit-feather-rp2040-adalogger/overview  
Adafruit BMP390 Barometer/Altimeter - https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/overview  
Adafruit Mini GPS PA1010D - https://learn.adafruit.com/adafruit-mini-gps-pa1010d-module/overview  
Adafruit PCF8523 Real Time Clock - https://learn.adafruit.com/adafruit-pcf8523-real-time-clock/overview  
Adafruit LSM6DSOX + LIS3MDL Accelerometer/Magnetometer - https://learn.adafruit.com/st-9-dof-combo/overview  