#!/usr/bin/env python3
import serial
import time
import numpy as np
import RPi.GPIO as GPIO
import asyncio

sensor_L = 4
sensor_M = 17
sensor_R = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor_L,GPIO.IN)
GPIO.setup(sensor_M,GPIO.IN)
GPIO.setup(sensor_R,GPIO.IN)


async def IRcheck(sensor):
	await asyncio.sleep(0.5)
	irValue = GPIO.input(sensor)
	if irValue:
		print(f"NOT DETECTED Sensor {sensor}")
				
	else:
		print(f"DETECTED Sensor {sensor}")

async def main():
	print('IR Sensor Ready')

	try:
		while True:
			print('_______________________')
			await IRcheck(sensor_L)
			await IRcheck(sensor_M)
			await IRcheck(sensor_R)
	except KeyboardInterrupt:
		GPIO.cleanup()

asyncio.run(main())