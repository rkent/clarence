#!/usr/bin/env python3
# Adapted from https://forums.raspberrypi.com/viewtopic.php?t=77775
''' For use on RPi 5, 
sudo apt remove python3-rpi.gpio
pip3 install --break-system-packages rpi-lgpio
'''

import os
import time
import sys
print(sys.path)

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
LEDs = (23, 24, 25)
for led in LEDs:
	GPIO.setup(led, GPIO.OUT)

#Setup variables for user input
led_choice = 0
count = 0

os.system('clear')

print("Which LED you want to blink?")
print("0: Red?")
print("1: Green?")
print("2: Yellow?")
print("3: All?")
led_choice = int(input("Make your choice: "))

if led_choice < 4:
	os.system('clear')
	count = int(input("How many times you want it to blink?: "))
	if led_choice == 3:
		leds = (0, 1, 2)
	else:
		leds = (led_choice,)
	while count > 0:
		for led in leds:
			GPIO.output(LEDs[led], GPIO.HIGH)
		time.sleep(1)
		for led in leds:
			GPIO.output(LEDs[led], GPIO.LOW)
		time.sleep(1)
		count = count - 1

GPIO.cleanup()
