This project is a basic data logger based on STM32F103C8T6 (bluepill) microcontroller.

Features:
1. 4 digital input channels.
2. 4 analog channels with multipliers (1x 10x 100x).
3. Writes stuff onto SD card.
4. Integrated RTC (microcontroller has RTC peripheral and bluepill has 32khz crystal).

Planned:
1. Code refactor (it is a mess).
2. Temperature logging (either 1-wire or I2C or both).
3. Flexible logging settings (but it's kinda awkward to switch on 4 key keyboard).
4. Wifi support?

Project was built for purpose of testing PIR sensors from GSM alarm project since they had lots of false triggering in regular time intervals, so i had to make something to monitor their behavior.

Materials: 
1. STM32F103C8T6 microcontroller on a board (blue pill).
2. HC165-based serial keyboard.
3. HD44780-based serial (I2C) LCD.
4. SD card SPI module.
5. CR2032 battery holder.
6. Onboard male and female connectors
7. SD card with FAT filesystems (or any that fatfs can read).
8. Lotsa wires.
 
Schematics and video will be soon.
Project pics: http://imgur.com/a/ej2eB

