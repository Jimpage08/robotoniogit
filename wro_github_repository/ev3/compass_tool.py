#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait
from time import sleep, time
from json import loads

class CompassTool:
    def __init__(self, ev3, sensor, filename="cnf.txt"):
        self.ev3 = ev3
        self.sensor = sensor
        self.cnf_file = filename
        self.orientations = None
        self.orientations_1 = [0, 90, 180, 270]
        self.orientations_2 = [0, 90, 180, 270]
        self.read_cnf_file()
        self.current_target_index = 0
        
    def get_current_target(self):
        return self.orientations[self.current_target_index]

    def turn(self, direction=1):
        self.current_target_index = (self.current_target_index + direction) % 4
        
    def print(self):
        if self.orientations is None:
            self.ev3.screen.print("Must Init Orientations")
            self.ev3.screen.print("CENTER: init")
            while Button.CENTER not in self.ev3.buttons.pressed():
                pass
            self.init_orientation()
        while Button.DOWN not in self.ev3.buttons.pressed():
            compass_input = self.heading()
            self.ev3.screen.clear()
            for orient in self.orientations:
                self.ev3.screen.print(str(orient))    
            self.ev3.screen.print("Heading: " + str(compass_input))
            self.ev3.screen.print("STOP: DOWN")
            wait(300)

    def heading(self):
        return self.sensor.read("COMPASS")[0]

    def calibrate_orientations(self):
        
        for index in range(4):
            for j in range(2):
                while True:
                    heading = self.heading()
                    self.ev3.screen.clear()
                    self.ev3.screen.print("")
                    self.ev3.screen.print(str(self.orientations_1))
                    self.ev3.screen.print(str(self.orientations_2))
                    self.ev3.screen.print(str(heading))
                    self.ev3.screen.print("or_" + str(j+1) + ": CENTER (index:" + str(index) + ")")
                    if Button.CENTER in self.ev3.buttons.pressed():
                        if j == 0:
                            self.orientations_1[index] = heading
                        else:
                            self.orientations_2[index] = heading
                        self.ev3.speaker.beep(duration=300)
                        break
                    wait(300)
        self.ev3.screen.clear()
        self.ev3.screen.print("")
        self.ev3.screen.print("Cal/ted values:")
        self.ev3.screen.print(str(self.orientations_1))
        self.ev3.screen.print(str(self.orientations_2))
        self.ev3.screen.print("Write: CENTER")
        self.ev3.screen.print("Cancel: DOWN")
        while True:
            if Button.DOWN in self.ev3.buttons.pressed():
                self.read_cnf_file()
                self.ev3.screen.clear()
                self.ev3.screen.print("Calibration canceled")
                self.ev3.screen.print("Wait 2\"")
                wait(2000)
                break
            if Button.CENTER in self.ev3.buttons.pressed():
                self.write_cnf_file()
                self.ev3.screen.clear()
                self.ev3.screen.print("Calibration saved")
                self.ev3.screen.print("Wait 2\"")
                wait(2000)
                break
            
    def read_cnf_file(self):
        with open(self.cnf_file, 'r') as f:
            for line in f:
                # print(line)
                dict_line = loads(line)
                if dict_line["data"] == "compass_1":
                    self.orientations_1[0] = dict_line['a']
                    self.orientations_1[1] = dict_line['b']
                    self.orientations_1[2] = dict_line['c']
                    self.orientations_1[3] = dict_line['d']
                elif dict_line["data"] == "compass_2":
                    self.orientations_2[0] = dict_line['a']
                    self.orientations_2[1] = dict_line['b']
                    self.orientations_2[2] = dict_line['c']
                    self.orientations_2[3] = dict_line['d']

    def write_cnf_file(self):
        replacement = ""
        # read file to change the values
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] == "compass_1":
                    dict_line["a"] = self.orientations_1[0]
                    dict_line["b"] = self.orientations_1[1]
                    dict_line["c"] = self.orientations_1[2]
                    dict_line["d"] = self.orientations_1[3]
                elif dict_line["data"] == "compass_2":
                    dict_line["a"] = self.orientations_2[0]
                    dict_line["b"] = self.orientations_2[1]
                    dict_line["c"] = self.orientations_2[2]
                    dict_line["d"] = self.orientations_2[3]
                replacement = replacement + str(dict_line) + "\n"
        sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

    def init_orientation(self):
        index = 0
        or1_index = 0
        heading = self.heading()
        for ort in self.orientations_1:
            ranges = []
            min_value = (359 + ort - 15) % 360
            max_value = (ort + 15) % 360
            if min_value < max_value:
                ranges.append((min_value, max_value))
            elif min_value > max_value:
                ranges.append((min_value, 360))
                ranges.append((0, max_value))                
            for r in ranges:
                if r[0] < heading < r[1]:                
                    or1_index = index
                    break
            index += 1
        index = 0
        or2_index = 0
        for ort in self.orientations_2:
            ranges = []
            min_value = (359 + ort - 15) % 360
            max_value = (ort + 15) % 360
            if min_value < max_value:
                ranges.append((min_value, max_value))
            elif min_value > max_value:
                ranges.append((min_value, 360))
                ranges.append((0, max_value))                
            for r in ranges:
                if r[0] < heading < r[1]:                  
                    or2_index = index
                    break
            index += 1
        or1_error = self.orientations_1[or1_index] - heading
        or2_error = self.orientations_2[or2_index] - heading
        if abs(or2_error) < abs(or1_error):
            self.orientations = self.orientations_2
            self.current_target_index = or2_index
        else:
            self.orientations = self.orientations_1
            self.current_target_index = or1_index
        
        
