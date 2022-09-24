#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from time import sleep
from pybricks.iodevices import UARTDevice
from time import sleep, time
from car import Car

car = Car()
car.menu()
# car.t_read_color()
# car.stop_to_line(speed=400, delay=50)
# car.follow_compass()
# car.test_steering()
# car.main()
# car.check_communication()
# car.calibrate_orientations()
# while True:
#     car.read_compass()
#     wait(1000)