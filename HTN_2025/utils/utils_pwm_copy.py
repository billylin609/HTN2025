#!/usr/bin/env python3
"""
Copyright (c) 2025, BlackBerry Limited. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Motor Driver Module for QNX using rpi_gpio.
 
This module demonstrates how to control the motor driver using a PWM signal on the enable (EN)
pin for speed control and digital outputs for motor direction. The DRV8833 motor driver’s speed
is controlled via a PWM signal on the enable (nSLEEP) pin, while the other pins set the motor
direction.
  
IMPORTANT:
  The motor enable (EN) pin MUST be configured for PWM output so that adjusting the duty cycle
  changes the motor speed.
"""
 
import rpi_gpio as GPIO  # Raspberry Pi GPIO module
import time
import sys


class MotorBase:
    def __init__(self, method='pwm', pwm_freq=50):
        match method:
            case 'pwm': 
                # GPIO pin assignments (using BCM numbering)
                self.MOTOR_L_PIN   = 12    # Left Motor
                self.MOTOR_R_PIN   = 13   # Right Motor
                
                # PWM frequency in Hz for motor speed control
                self.PWM_FREQ = pwm_freq
                
                self._hardware_pwm_init()
            case _:
                raise SystemExit(1)    


    def _hardware_pwm_init(self):
        # Use BCM numbering for GPIO pins.
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.MOTOR_L_PIN, GPIO.OUT)
        GPIO.setup(self.MOTOR_R_PIN, GPIO.OUT)
        
        # Configure Hardware PWM Pin
        self.pwm_l = GPIO.PWM(self.MOTOR_L_PIN, self.PWM_FREQ)
        self.pwm_r = GPIO.PWM(self.MOTOR_R_PIN, self.PWM_FREQ)
        
        # Initialize PWM idle Speed
        self.pwm_l.start(7.6)
        self.pwm_r.start(7.6)


    def stop(self):
        self.pwm_l.ChangeDutyCycle(7.6)
        self.pwm_r.ChangeDutyCycle(7.6)

    # Speed in Percentage(0.0-100.0)
    def set_left_speed(self, speed):
        self.pwm_l.ChangeDutyCycle(speed)


    # Speed in Percentage(0.0-100.0)
    def set_right_speed(self, speed):
        self.pwm_r.ChangeDutyCycle(speed)



control = MotorBase()
while(1):
    control.set_right_speed(10)
    control.set_left_speed(10)
