#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from pybricks.iodevices import UARTDevice
from pybricks.iodevices import Ev3devSensor
from time import sleep, time
# from line_tool import LineTool
from compass_tool import CompassTool
from jetson import Jetson
from line_tool import LineTool
from PID import PID
from threading import Thread

PORT_COLOR = Port.S1
PORT_UART = Port.S2
PORT_COMPASS = Port.S4
PORT_DC = Port.C
PORT_STEERING = Port.B

STEERING_THRESHOLD = 30

def change_range(value, in_min, in_max, out_min, out_max):
    scaled_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return int(scaled_value)

class Car:
    def __init__(self, filename="cnf.txt"):
        self.cnf_filename = filename
        self.ev3 = EV3Brick()
        self.ev3.screen.set_font(Font(size=14, bold=True))
        try:
            self.dc_motor = Motor(PORT_DC)
        except:
            print("Problem with Port ", PORT_DC)
            # exit()
            return
        try:
            self.steering_motor = Motor(PORT_STEERING, positive_direction=Direction.COUNTERCLOCKWISE)
        except:
            print("Problem with Port ", PORT_STEERING)
            # exit()
            return
        self.steering_threshold = STEERING_THRESHOLD
        self.steering_delay = 5
        self.steering_last = 0
        self.current_steer = 0
        try:
            self.compass_sensor = Ev3devSensor(PORT_COMPASS)
        except:
            print("Problem with Port ", PORT_COMPASS)
            exit()
        try:
            self.color_sensor =  ColorSensor(PORT_COLOR)
            self.line_tool = LineTool(self.ev3, self.color_sensor, filename)
        except:
            print("Problem with Port ", PORT_COLOR)
            exit()
        
        self.serial = None
        self.jetson = None
        self.connected = False
        # self.run_num = run
        self.uart_connection()
        self.zero_heading = 0
        self.timer = StopWatch()
        self.compass_tool = CompassTool(self.ev3, self.compass_sensor, self.cnf_filename)
        
        self.round_direction = 0
        self.distance_threshold = 15
        self.turn = False
        self.run_color_thread = True
        
    def uart_connection(self):
        self.connected = False
        while not self.connected:
            try:
                self.serial = UARTDevice(PORT_UART, baudrate=115200)
                if self.serial is not None:
                    self.ev3.screen.clear()
                    self.ev3.screen.print("")
                    self.ev3.screen.print("")
                    self.ev3.screen.print("CONNECTED!")
                    self.connected = True
                else:
                    self.ev3.screen.clear()
                    self.ev3.screen.print("")
                    self.ev3.screen.print("")
                    self.ev3.screen.print("FAIL TO CONNECT")
            except:
                self.ev3.screen.clear()
                self.ev3.screen.print("")
                self.ev3.screen.print("")
                self.ev3.screen.print("FAIL TO CONNECT")
            wait(500)
        self.jetson = Jetson(self.ev3, self.serial)
        wait(3000)

    

    def print_list(self, txt_list, selected=-1):
        self.ev3.screen.clear()
        for i, txt in enumerate(txt_list):
            if i == selected:
                txt = "> " + txt
            self.ev3.screen.print(txt)

    def menu(self):
        main_options = cnf_options = {}
        # Run 1 Options
        run1_options = {"GO":1, "BACK":"main_options"}
        # Run 2 Options
        run2_options = {"GO":2, "BACK":"main_options"}
        # Info options
        info_options = {"Read Jetson":3, "Read Color":4, "Read Compass":5, "BACK":"main_options"}
        # Configuration options
        cnf_options = {"Calibtate Color":7, "Calibrate Orientation":8, "Get Photos":9, "BACK":"main_options"}
        # Main menu options
        main_options = {"RUN 1":run1_options, "RUN 2":run2_options, "INFO":info_options, "CONFIGURATION":cnf_options}
        
        current_options = main_options
        exit_menu = False
        request = 0
        while not exit_menu:
            selected = 1
            option_list = list(current_options.keys())
            if self.serial is None:
                option_list.insert(0, "Not Connected")
            else:
                option_list.insert(0, "CONNECTED")
            
            while True:
                self.print_list(option_list, selected=selected)
                if Button.DOWN in self.ev3.buttons.pressed():
                    #print("next")
                    selected += 1
                    if selected >= len(option_list):
                        selected = len(option_list) - 1
                    sleep(0.5)
                if Button.UP in self.ev3.buttons.pressed():
                    #print("previous")
                    selected -= 1
                    if selected < 1:
                        selected = 1
                    sleep(0.5)
                if Button.CENTER in self.ev3.buttons.pressed():
                    #print("hit")
                    option = current_options[option_list[selected]]
                    #print(option)
                    sleep(0.5)
                    if type(option) is dict:
                        # Next level of options
                        current_options = option
                        break
                    else:
                        if option_list[selected] == "BACK":
                            if current_options["BACK"] == "main_options":
                                current_options = main_options
                            elif current_options["BACK"] == "cnf_options":
                                current_options = cnf_options
                        request = option
                        #print("option:" + str(option))
                        exit_menu = True
                        break
                    
                # To allow time for OLED refresh
                sleep(0.1)
        self.main(request)

    def main(self, request=0):
        '''
        Via the main method runs all the car's processes
        Through the menu method in the end of the process, it is called recursively
        '''
        if request == 1:
            if not self.connected:
                self.uart_connection()
            self.jetson.send_and_confirm([10])  
            print("RUN1!")
            # RUN 1   
            self.run_1()
        elif request == 2:
            if not self.connected:
                self.uart_connection()
            self.jetson.send_and_confirm([11])
            # RUN 2 
            self.run()
        elif request == 3:
            self.read_jetson()
        elif request == 4:
            self.read_color()
        elif request == 5:
            self.read_compass()
        elif request == 7:
            self.calibrate_color()
        elif request == 8:
            self.calibrate_orientations()
        elif request == 9:
            if not self.connected:
                self.uart_connection()
            self.jetson.send_and_confirm([12])
            self.get_photos()
        request=0
        self.menu()

    def run_1(self, speed=300, rounds=3, max_steering=35):
        self.round_direction = 0
        # self.compass_tool.init_orientation()
        compass_target = self.compass_tool.heading() # self.compass_tool.get_current_target()
        self.steering_motor.reset_angle(0)
        
        self.steer(0)
        
        # wait(200)
        last_turn_time = 0
        turn_delay = 3000
        t = Thread(target=self.t_read_color, args=())
        t.start()
        rounds_passed = 0
        turns_passed = 0
        self.dc_motor.run(speed)
        self.jetson.send_data([0])
        self.ev3.light.off()
        max_compass_steering = 30
        last_turn = 0
        last_jetson_steering = 0
        jetson_fr = 0
        last_compass = 0
        compass_delay = 5
        sample_time = 250
        self.ev3.light.off()
        while rounds_passed < rounds:
            
            # to stop process when you press DOWN
            if Button.DOWN in self.ev3.buttons.pressed():
                print("Down pressed")
                self.dc_motor.stop()
                self.run_color_thread = False
                self.steer(0)
                self.jetson.send_data([6])
                self.ev3.light.off()
                wait(3000)
                self.ev3.screen.clear()
                self.ev3.screen.print("UP: Restart RUN 2")
                while Button.UP not in self.ev3.buttons.pressed():
                    pass
                return None

            current_ms = self.timer.time()
            if current_ms - last_turn > 3000:
                max_compass_turn = 30
                self.ev3.light.off()
            # Looking for turn (via color sensor)
            if self.turn:
                last_turn = current_ms
                self.ev3.light.on(Color.GREEN)
                self.turn = False
                print("Turn", self.round_direction)
                
                max_compass_steering = 45

                turns_passed += 1
                if turns_passed > 3:
                    turns_passed = 0
                    rounds_passed += 1
                self.compass_tool.turn(self.round_direction)

            # Jetson time!
            jetson_data = self.jetson.get_data()
            if jetson_data is not None:
                self.ev3.light.off()
                j_start = self.timer.time()
                request = jetson_data[0]                
                # self.ev3.screen.print(str(jetson_data))
                if request == 5 and self.round_direction != 0:
                    steering = max_steering
                    # self.steer(steering * self.round_direction) # steering to avoid front wall
                    self.smooth_steering(new_steer=steering * self.round_direction, max_angle=15)
                    last_jetson_steering = current_ms
                if request == 2:
                    steering = max(-max_steering, min(max_steering, jetson_data[1]))
                    self.smooth_steering(new_steer=steering, max_angle=20)
                    # self.steer(steering) # steering to avoid side wall or pillar
                    last_jetson_steering = current_ms
                # print("Jetson steerin time", self.timer.time())
                # print("j time:", self.timer.time() - j_start, jetson_data)

            elif current_ms - last_jetson_steering > sample_time and current_ms - last_compass > compass_delay:
                last_compass = current_ms
                c_start = self.timer.time()
                
                self.follow_compass(target=compass_target, max_steering=max_compass_steering)
                # print("Compass steering time", self.timer.time() - c_start)
                
        self.steer(0)
        starting_angle = abs(self.dc_motor.angle())
        while abs(self.dc_motor.angle()) - starting_angle < 1000:
            
            self.follow_compass()
            
        self.dc_motor.stop()

    def read_color(self):
        self.line_tool.print()

    def read_jetson(self):
        self.jetson.listen()

    def read_compass(self):
        self.compass_tool.print()

    def calibrate_orientations(self):
        self.compass_tool.calibrate_orientations()

    def calibrate_color(self):
        self.line_tool.calibrate_lines()

    def get_photos(self):
        while not self.connected or self.serial is None:
            self.ev3.screen.clear()
            self.ev3.screen.print("Not connected")
            self.ev3.screen.print("UP: to connect")
            self.ev3.screen.print("Check wires first")
            self.ev3.screen.print("Serial to port 2")
            self.ev3.screen.print("DOWN: STOP")
            if Button.UP in self.ev3.buttons.pressed():
                self.uart_connection()
            if Button.DOWN in self.ev3.buttons.pressed():
                return None
            wait(300)
        self.ev3.screen.clear()
        self.ev3.screen.print("Connected!")
        
        while True:
            self.ev3.screen.clear()
            self.ev3.screen.print("CENTER: SHOT!")
            self.ev3.screen.print("DOWN: STOP")
            if Button.CENTER in self.ev3.buttons.pressed():
                self.jetson.send_data([1], open_sign='!')
                self.ev3.speaker.beep(duration=500)
            if Button.DOWN in self.ev3.buttons.pressed():
                self.jetson.send_data([0], open_sign='!')
                self.ev3.speaker.beep(duration=2000)
                break
            wait(300)
        
        self.ev3.screen.print("wait 3 seconds ...")
        wait(3000)

    def follow_jetson(self, max_steering=20):
        steering = 0
        stop_robot = False
        last_turn_time = 0
        turn_delay = 1500
        while not stop_robot:
            current_ms = self.timer.time()
            jetson_data = self.jetson.get_data()
            if jetson_data is not None:
                request = jetson_data[0]
                print("Request: ", jetson_data)
                if request == 2:
                    steering = max(-max_steering, min(max_steering, jetson_data[1]))
                elif request == 3:
                    self.dc_motor.stop()
                    stop_robot = True
                elif request == -4 and current_ms - last_turn_time > turn_delay:
                    last_turn_time = current_ms
                    # self.jetson.send_data(jetson_data) # to confirm
                    # turn compass left
                    self.turn_compass(direction=-1)
                    steering = 0
                elif request == 4 and current_ms - last_turn_time > turn_delay:
                    last_turn_time = current_ms
                    # self.jetson.send_data(jetson_data) # to confirm
                    # turn compass right
                    self.turn_compass(direction=1)
                    steering = 0
            self.smooth_steering(current_ms, new_steer=steering, max_angle=2)
            # self.steer(steering, current_ms)
                
    def turn_compass(self, direction=1, max_steering=40, angle_offset=14):
        # print("Turn compass " + str(direction))
        # print("Current compass index", self.compass_tool.current_target_index, ", pos:", self.compass_tool.orientations[self.compass_tool.current_target_index])
        self.compass_tool.turn(direction)
        new_compass_pos = self.compass_tool.orientations[self.compass_tool.current_target_index]
        # print("New compass index", self.compass_tool.current_target_index, ", pos:", new_compass_pos)
        
        ranges = []
        min_value = (359 + new_compass_pos - angle_offset) % 360
        max_value = (new_compass_pos + angle_offset) % 360
        if min_value < max_value:
            ranges.append((min_value, max_value))
        elif min_value > max_value:
            ranges.append((min_value, 360))
            ranges.append((0, max_value))
        # print(ranges)
        stop_turning = False
        turn_angle = max_steering * direction
        # self.steer(turn_angle)
        while not stop_turning:
            heading = self.compass_tool.heading()
            # print(heading)
            self.smooth_steering(new_steer=turn_angle, max_angle=5)
            for r in ranges:
                if r[0] < heading < r[1]:
                    stop_turning = True
                    break
        wait(self.steering_delay) # to ensure steering to 0
        self.steer(0)
        self.jetson.send_data([5])
        # print("stop turn")

    def follow_compass(self, target=None, kp=1, max_steering=35):
        if target is None:
            target = self.compass_tool.get_current_target()
        self.ev3.light.on(Color.ORANGE)
        compass_error = self.compass_tool.heading() - target
        if compass_error > 180:
            compass_error = compass_error - 360
        elif compass_error < -180:
            compass_error = 360 + compass_error
        steering = max(-max_steering, min(max_steering, -compass_error * kp))
        print("compass_error:", compass_error, "steering", steering)
        # self.steer(steering)
        
        self.smooth_steering(new_steer=steering, max_angle=10)

    def t_read_color(self):
        last_turn = 0
        turn_offset = 3000
        self.turn = False
        self.ev3.screen.clear()
        self.run_color_thread = True
        while self.run_color_thread:
            current_ms = time() * 1000
            line = self.line_tool.detect_line()
            
            if line is not None and current_ms - last_turn > turn_offset:
                # print(current_ms, line)
                # self.ev3.screen.print(str(line))
                last_turn = current_ms
                if self.round_direction == 0:
                    current_heading = self.compass_tool.heading()
                    if line == "blue":
                        self.round_direction = -1
                        self.compass_tool.orientations = self.compass_tool.orientation_2
                    else:
                        self.round_direction = 1
                        self.compass_tool.orientations = self.compass_tool.orientation_1
                    or_index = 0
                    for ort in self.compass_tool.orientations:
                        ranges = []
                        min_value = (359 + ort - 15) % 360
                        max_value = (ort + 15) % 360
                        if min_value < max_value:
                            ranges.append((min_value, max_value))
                        elif min_value > max_value:
                            ranges.append((min_value, 360))
                            ranges.append((0, max_value))                
                        for r in ranges:
                            if r[0] < current_heading < r[1]:                  
                                self.compass_tool.current_target_index = index
                                break
                        index += 1

                if line == "blue" and self.round_direction == -1:
                    self.turn = True
                elif line == "orange" and self.round_direction == 1:
                    self.turn = True
        print("Thread killed")
            


    def run(self, speed=300, rounds=4, max_steering=35):
        self.round_direction = 0
        # self.compass_tool.init_orientation()
        compass_target = self.compass_tool.heading() # self.compass_tool.get_current_target()
        self.steering_motor.reset_angle(0)
        
        self.steer(0)
        
        # wait(200)
        last_turn_time = 0
        turn_delay = 3000
        t = Thread(target=self.t_read_color, args=())
        t.start()
        rounds_passed = 0
        turns_passed = 0
        self.dc_motor.run(speed)
        self.jetson.send_data([0])
        self.ev3.light.off()
        max_compass_steering = 30
        last_turn = 0
        last_jetson_steering = 0
        jetson_fr = 0
        last_compass = 0
        compass_delay = 5
        sample_time = 250
        self.ev3.light.off()
        while rounds_passed < rounds:
            
            # to stop process when you press DOWN
            if Button.DOWN in self.ev3.buttons.pressed():
                print("Down pressed")
                self.dc_motor.stop()
                self.run_color_thread = False
                self.steer(0)
                self.jetson.send_data([6])
                self.ev3.light.off()
                wait(3000)
                self.ev3.screen.clear()
                self.ev3.screen.print("UP: Restart RUN 2")
                while Button.UP not in self.ev3.buttons.pressed():
                    pass
                return None

            current_ms = self.timer.time()
            if current_ms - last_turn > 3000:
                max_compass_turn = 30
                self.ev3.light.off()
            # Looking for turn (via color sensor)
            if self.turn:
                last_turn = current_ms
                self.ev3.light.on(Color.GREEN)
                self.turn = False
                print("Turn", self.round_direction)
                
                max_compass_steering = 45

                turns_passed += 1
                if turns_passed > 3:
                    turns_passed = 0
                    rounds_passed += 1
                self.compass_tool.turn(self.round_direction)

            # Jetson time!
            jetson_data = self.jetson.get_data()
            if jetson_data is not None:
                self.ev3.light.off()
                j_start = self.timer.time()
                request = jetson_data[0]                
                # self.ev3.screen.print(str(jetson_data))
                if request == 5 and self.round_direction != 0:
                    steering = max_steering
                    # self.steer(steering * self.round_direction) # steering to avoid front wall
                    self.smooth_steering(new_steer=steering * self.round_direction, max_angle=15)
                    last_jetson_steering = current_ms
                if request == 2:
                    steering = max(-max_steering, min(max_steering, jetson_data[1]))
                    self.smooth_steering(new_steer=steering, max_angle=20)
                    # self.steer(steering) # steering to avoid side wall or pillar
                    last_jetson_steering = current_ms
                # print("Jetson steerin time", self.timer.time())
                # print("j time:", self.timer.time() - j_start, jetson_data)

            elif current_ms - last_jetson_steering > sample_time and current_ms - last_compass > compass_delay:
                last_compass = current_ms
                c_start = self.timer.time()
                
                self.follow_compass(target=compass_target, max_steering=max_compass_steering)
                # print("Compass steering time", self.timer.time() - c_start)
                
        self.steer(0)
        starting_angle = abs(self.dc_motor.angle())
        while abs(self.dc_motor.angle()) - starting_angle < 1000:
            
            self.follow_compass()
            
        self.dc_motor.stop()


      
    def test_steering(self):
        # self.steer(60)
        # wait(2000)
        # self.steer(0)
        # wait(2000)
        # self.steer(-60)
        # wait(2000)
        # self.steer(0)
        while True:
            self.smooth_steering(new_steer=50, max_angle=2)
        

    def smooth_steering(self, current_time=None, new_steer=0, max_angle=10):
        if new_steer == self.current_steer:
            return None
        if current_time is None:
            current_time = self.timer.time()
        if current_time - self.steering_last > self.steering_delay:
            angle = new_steer - self.current_steer
            if angle == 0:
                return None
            if abs(angle) > max_angle:
                self.current_steer = int(self.current_steer + max_angle * (angle/abs(angle)))
                # smooth_steer = int(self.current_steer + max_angle * (angle/abs(angle)))
            else:
                self.current_steer = new_steer
            print("new_steer:", new_steer, "real steering:", self.current_steer)
            self.steering_motor.run_target(speed=1000, target_angle=self.current_steer, wait=False)
            self.steering_last = current_time

    def stop_to_line(self, speed=300, delay=0):
        go = True
        while go:
            self.dc_motor.run(speed)
            line = self.line_tool.detect_line()
            while line is None:
                wait(delay)
                line = self.line_tool.detect_line()
            self.dc_motor.stop()
            self.ev3.screen.clear()
            self.ev3.screen.print(line)
            self.ev3.screen.print("DOWN: exit")
            self.ev3.screen.print("UP: again")
            while True:
                if Button.DOWN in self.ev3.buttons.pressed():
                    go = False
                    break
                if Button.UP in self.ev3.buttons.pressed():
                    break
            


    def steer(self, u, current_time=None):
        if abs(u - self.current_steer) < 2:
            return None
        if current_time is None:
            current_time = self.timer.time()
        if current_time - self.steering_last > self.steering_delay:
            degrees = min(max(u, -self.steering_threshold), self.steering_threshold)
            self.steering_motor.run_target(speed=500, target_angle=-degrees, wait=True)
            self.steering_last = current_time
            self.current_steer = degrees
        # self.steering_motor.run_target(speed=50, target_angle=degrees, wait=True)

    def heading(self):
        return self.compass_sensor.read("COMPASS")[0]

    def check_communication(self):
        while True:
            self.jetson.send_data([0, 255])
            wait(500)
            self.ev3.screen.print(str(self.jetson.get_data()))
            wait(500)

