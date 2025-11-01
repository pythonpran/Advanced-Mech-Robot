import RPi.GPIO as GPIO
import time

# BCM numbering
BUMP_PINS = [7, 8, 11, 20, 21, 26]

def bump_callback(channel):
    """Called automatically whenever a bumper changes state."""
    state = GPIO.input(channel)
    if state == 0:
        print(f"Bump sensor on GPIO {channel} PRESSED")
    else:
        print(f"Bump sensor on GPIO {channel} RELEASED")

def setup_bumps():
    GPIO.setmode(GPIO.BCM)
    for pin in BUMP_PINS:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Detect both press (FALLING) and release (RISING)
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=bump_callback, bouncetime=50)

def main():
    setup_bumps()
    print("Monitoring bump sensors on GPIO 7, 8, 11, 20, 21, 26.  Press Ctrl+C to exit.")
    try:
        while True:
            # do other non-blocking work here
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()