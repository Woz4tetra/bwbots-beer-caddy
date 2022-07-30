import time
import math
import asyncio
from RPi import GPIO


from lib.exceptions import ShutdownException


class GpioManager:
    def __init__(self, logger, config):
        self.logger = logger
        self.config = config

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.config.led_out, GPIO.OUT)
        GPIO.setup(self.config.fan_out, GPIO.OUT)
        GPIO.setup(self.config.lidar_out, GPIO.OUT)
        GPIO.setup(self.config.button_in, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button pin set as input w/ pull-up

        self.led_pwm = GPIO.PWM(self.config.led_out, 50)  # Initialize PWM with 100Hz frequency
        self.led_duty_cycle = 0

        self.fan_pwm = GPIO.PWM(self.config.fan_out, 50)
        self.fan_duty_cycle = 0

        self.led_pwm.start(self.led_duty_cycle)
        self.fan_pwm.start(self.fan_duty_cycle)

        self.shutdown_timer = time.time()

        self.led_cycle_direction = False
        self.prev_button_state = False
        self.button_state = False
        self.time_print_out = None

        self.logger.info("GPIO Hub initialized")


    def start(self):
        self.set_fan(100)
        self.set_lidar_pin(True)

    def is_button_pressed(self):
        return not GPIO.input(self.config.button_in)

    def set_lidar_pin(self, state):
        GPIO.output(self.config.lidar_out, GPIO.HIGH if state else GPIO.LOW)

    def set_led(self, duty_cycle):
        self.led_duty_cycle = duty_cycle
        self.led_pwm.ChangeDutyCycle(self.led_duty_cycle)

    def set_fan(self, duty_cycle):
        self.fan_duty_cycle = duty_cycle
        self.fan_pwm.ChangeDutyCycle(self.fan_duty_cycle)

    def log_shutdown_timer(self, time_offset):
        time_print_out = math.ceil(self.config.shutdown_time_s - time_offset)
        if time_print_out != self.time_print_out:
            self.logger.info("\t%s..." % time_print_out)
            self.time_print_out = time_print_out

    async def update(self):
        self.button_state = self.is_button_pressed()
        if self.button_state != self.prev_button_state:
            if self.button_state:
                self.logger.info("Shutdown button is being held. Shutting down in...")
                self.shutdown_timer = time.time()
            else:
                self.time_print_out = None
                self.logger.info("Shutdown button released. Shutdown cancelled")

            self.prev_button_state = self.button_state
        if self.button_state:
            if self.led_cycle_direction:
                cycle = self.led_duty_cycle + self.config.led_pwm_rate
                if cycle > 100:
                    cycle = 100
                    self.led_cycle_direction = False
            else:
                cycle = self.led_duty_cycle - self.config.led_pwm_rate
                if cycle < 0:
                    cycle = 0
                    self.led_cycle_direction = True
            self.set_led(cycle)

            time_offset = time.time() - self.shutdown_timer
            self.log_shutdown_timer(time_offset)

            if time_offset > self.config.shutdown_time_s:
                self.logger.info("Starting shutdown routine")
                raise ShutdownException
        else:
            self.set_led(100)
        await asyncio.sleep(0.05)

    def stop(self):
        # self.led_pwm.stop()  # leave LED on until the computer shuts down
        self.fan_pwm.stop()
        GPIO.cleanup()
