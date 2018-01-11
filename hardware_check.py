import RPi.GPIO as GPIO
import time

def make_LED(color):
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


def setup_LED():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(led_green_pin, GPIO.OUT)
    GPIO.setup(led_red_pin,   GPIO.OUT)
    return


led_green_pin = 23
led_red_pin = 24
setup_LED()

GPIO.setmode(GPIO.BCM)

GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    print('Press any button, or both simultaniously. Press crtl+c to end program.')
    while True:
        input_state = GPIO.input(18)
        input_state_2 = GPIO.input(17)
        if input_state == 0 and input_state_2 == 1:
            print('You are pressing button 1, LED should be green')
            make_LED("GREEN")
        if input_state == 1  and input_state_2 == 0:
            print('You are pressing button 2, LED should be orange')
            make_LED("ORANGE")
        if input_state == 0 and input_state_2 == 0:
            print('You are pressing both butons, LED shoud be red')
            make_LED("RED")
        time.sleep(0.2)
except KeyboardInterrupt:
    make_LED("OFF")
    GPIO.cleanup()
    print "\nBye"

