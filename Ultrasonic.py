import RPi.GPIO as GPIO
import time

# Pin setup (BCM numbering)
TRIG_1 = 27
ECHO_1 = 22
TRIG_2 = 23
ECHO_2 = 24
TRIG_3 = 25
ECHO_3 = 5
TRIG_4 = 6
ECHO_4 = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIG_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)
GPIO.setup(TRIG_3, GPIO.OUT)
GPIO.setup(ECHO_3, GPIO.IN)
GPIO.setup(TRIG_4, GPIO.OUT)
GPIO.setup(ECHO_4, GPIO.IN)

def get_distance(TRIG,ECHO):
    """Measure distance using HC-SR04 (in cm)."""
    # Ensure trigger is LOW
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    # Send 10µs pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo start
    start = time.time()
    timeout = start + 0.02  # 20ms safety timeout
    while GPIO.input(ECHO) == 0 and time.time() < timeout:
        start = time.time()

    # Wait for echo end
    stop = time.time()
    timeout = stop + 0.02
    while GPIO.input(ECHO) == 1 and time.time() < timeout:
        stop = time.time()

    # Compute distance
    elapsed = stop - start
    distance_cm = (elapsed * 34300) / 2
    return round(distance_cm, 2)

try:
    while True:
        dist_1 = get_distance(TRIG_1,ECHO_1)
        print(f"Distance 1: {dist_1} cm")
        dist_2 = get_distance(TRIG_2,ECHO_2)
        print(f"Distance 2: {dist_2} cm")
        dist_3 = get_distance(TRIG_3,ECHO_3)
        print(f"Distance 3: {dist_3} cm")
        dist_4 = get_distance(TRIG_4,ECHO_4)
        print(f"Distance 4: {dist_4} cm")
        print(f"__________________________")
        time.sleep(0.5)  # non-blocking style loop

except KeyboardInterrupt:
    print("\nExiting.")
finally:
    GPIO.cleanup()