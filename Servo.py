import pigpio, time
pi = pigpio.pi()
SERVO_PIN = 13

while True:
    for pulse in range(500,2500,200):
        pi.set_servo_pulsewidth(SERVO_PIN, pulse)
        time.sleep(0.5)