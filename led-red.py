import RPi.GPIO as GPIO

def make_LED(color, led_green_pin, led_red_pin):
    if color == "RED":
        GPIO.output(led_green_pin, GPIO.LOW)
        GPIO.output(led_red_pin,   GPIO.HIGH)
    elif color == "GREEN":
        GPIO.output(led_green_pin, GPIO.HIGH)
        GPIO.output(led_red_pin,   GPIO.LOW)
    elif color == "ORANGE":
        GPIO.output(led_green_pin, GPIO.HIGH)
        GPIO.output(led_red_pin,   GPIO.HIGH)
    else:
        GPIO.output(led_green_pin, GPIO.LOW)
        GPIO.output(led_red_pin,   GPIO.LOW)
    return


def setup_LED(led_green_pin, led_red_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(led_green_pin, GPIO.OUT)
    GPIO.setup(led_red_pin,   GPIO.OUT)
    return

def main():
    led_green_pin = 23
    led_red_pin = 24
    setup_LED(led_green_pin, led_red_pin)
    GPIO.setmode(GPIO.BCM)
    make_LED("RED", led_green_pin, led_red_pin)

if __name__ == "__main__":
    main()
