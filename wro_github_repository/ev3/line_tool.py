#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait
from time import sleep, time
from json import loads

class LineTool:
    def __init__(self, ev3, color_sensor, filename="cnf.txt"):
        self.ev3 = ev3
        self.sensor = color_sensor
        self.cnf_file = filename
        orange = {"hmin":0, "smin":0, "vmin":0, "hmax":0, "smax":0, "vmax":0, "turn":1}
        blue = {"hmin":0, "smin":0, "vmin":0, "hmax":359, "smax":255, "vmax":255, "turn":-1}
        self.lines = {"orange":orange, "blue":blue}
        self.read_cnf_file()
     
    def print(self):  
        while Button.DOWN not in self.ev3.buttons.pressed():
                line = self.detect_line()
                self.ev3.screen.clear()
                self.ev3.screen.print("Color: " + str(line))
                self.ev3.screen.print("STOP: DOWN")
                wait(300)

    def detect_line(self):
        h, s, v = self.hsv()
        for color_key in self.lines.keys():
            if self.lines[color_key]['hmin'] <= h <= self.lines[color_key]['hmax'] and \
                self.lines[color_key]['vmin'] <= v <= self.lines[color_key]['vmax']:
                return color_key
        return None

    def hsv(self):
        r, g, b = self.sensor.rgb()
        r, g, b = r / 100, g / 100, b / 100
        maxc = max(r, g, b)
        minc = min(r, g, b)
        v = maxc
        if minc == maxc:
            return 0.0, 0.0, v
        s = (maxc-minc) / maxc
        rc = (maxc-r) / (maxc-minc)
        gc = (maxc-g) / (maxc-minc)
        bc = (maxc-b) / (maxc-minc)
        if r == maxc:
            h = bc-gc
        elif g == maxc:
            h = 2.0+rc-bc
        else:
            h = 4.0+gc-rc
        h = (h/6.0) % 1.0
        return min(359, int(h*359)), min(255, int(s*255)), min(255, int(v*255))

    def read_cnf_file(self):
        color_names = self.lines.keys()
        with open(self.cnf_file, 'r') as f:
            for line in f:
                # print(line)
                dict_line = loads(line)
                if dict_line["data"] in color_names:
                    self.lines[dict_line["data"]]['hmin'] = dict_line['hmin']
                    self.lines[dict_line["data"]]['hmax'] = dict_line['hmax']
                    self.lines[dict_line["data"]]['smin'] = dict_line['smin']
                    self.lines[dict_line["data"]]['smax'] = dict_line['smax']
                    self.lines[dict_line["data"]]['vmin'] = dict_line['vmin']
                    self.lines[dict_line["data"]]['vmax'] = dict_line['vmax']
                    self.lines[dict_line["data"]]['turn'] = dict_line['turn']

    def write_cnf_file(self, color_name=None):
        color_names = []
        if color_name is None:
            color_names = self.lines.keys()
        else:
            color_names.append(color_name)
        replacement = ""
        # read file to change the values
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] in color_names:
                    dict_line["hmin"] = self.lines[dict_line["data"]]['hmin']
                    dict_line["hmax"] = self.lines[dict_line["data"]]['hmax']
                    dict_line["smin"] = self.lines[dict_line["data"]]['smin']
                    dict_line["smax"] = self.lines[dict_line["data"]]['smax']
                    dict_line["vmin"] = self.lines[dict_line["data"]]['vmin']
                    dict_line["vmax"] = self.lines[dict_line["data"]]['vmax']
                replacement = replacement + str(dict_line) + "\n"
        sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

    def calibrate_lines(self):
        
        color_names = self.lines.keys()
        for color_name in color_names:
            hmin = 359
            smin = vmin = 255
            hmax = smax = vmax = 0
            while Button.DOWN not in self.ev3.buttons.pressed():
                hsv = self.hsv()
                self.ev3.screen.clear()
                self.ev3.screen.print("")
                self.ev3.screen.print("CENTER to keep value")
                self.ev3.screen.print("DOWN for next color")
                self.ev3.screen.print("MIN: " + str(hmin) + " " + str(smin) + " " +  str(vmin))
                self.ev3.screen.print("MIN: " + str(hmax) + " " + str(smax) + " " +  str(vmax))
                self.ev3.screen.print(str(hsv))
                self.ev3.screen.print("Calibrate " + color_name)
                if Button.CENTER in self.ev3.buttons.pressed():
                    if hsv[0] < hmin:
                        hmin = hsv[0]
                    if hsv[1] < smin:
                        smin = hsv[1]
                    if hsv[2] < vmin:
                        vmin = hsv[2]
                    if hsv[0] > hmax:
                        hmax = hsv[0]
                    if hsv[1] > smax:
                        smax = hsv[1]
                    if hsv[2] > vmax:
                        vmax = hsv[2]
                    self.ev3.speaker.beep(duration=500)
                wait(300)
            self.ev3.screen.clear()
            self.ev3.screen.print("")
            self.ev3.screen.print("")
            self.ev3.screen.print("")
            self.ev3.screen.print("CENTER to save to file")
            self.ev3.screen.print("UP to ignore values")
            self.ev3.screen.print("Color: " + color_name)
            self.ev3.screen.print("MIN: " + str(hmin) + " " + str(smin) + " " +  str(vmin))
            self.ev3.screen.print("MAX: " + str(hmax) + " " + str(smax) + " " +  str(vmax))
            while True:
                if Button.UP in self.ev3.buttons.pressed():
                    self.ev3.screen.clear()
                    self.ev3.screen.print(color_name + " calibration canceled")
                    self.ev3.screen.print("Wait 2\"")
                    wait(2000)
                    break
                if Button.CENTER in self.ev3.buttons.pressed():
                    self.lines[color_name]['hmin'] = hmin
                    self.lines[color_name]['hmax'] = hmax
                    self.lines[color_name]['smin'] = smin
                    self.lines[color_name]['smax'] = smax
                    self.lines[color_name]['vmin'] = vmin
                    self.lines[color_name]['vmax'] = vmax
                    print("a", self.lines)
                    self.write_cnf_file(color_name)
                    print("b", self.lines)
                    self.ev3.screen.clear()
                    self.ev3.screen.print(color_name + " calibration saved")
                    self.ev3.screen.print("Wait 2\"")
                    wait(2000)
                    break

def main():
    ev3 = EV3Brick()
    color_sensor = ColorSensor(Port.S1)
    line_tool = LineTool(ev3, color_sensor, filename="cnf.txt")
    # line_tool.calibrate_lines()
    while True:
        
        if line_tool.detect_line() == "orange":
            ev3.screen.clear()
            ev3.screen.print('orange')
            wait(1000)
            ev3.screen.clear()
        if line_tool.detect_line() == "blue":
            ev3.screen.clear()
            ev3.screen.print('blue')
            wait(1000)
            ev3.screen.clear()
        #wait(200)

if __name__ == "__main__":
    main()
    