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

Schematics and video will be soon.
