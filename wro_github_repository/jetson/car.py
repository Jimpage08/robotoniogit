from distutils import dir_util

from scipy import inner
from components.camera import Camera
from components.distance import Distance
from components.ev3 import EV3
from cvtools.cvtool import CVTool
from cvtools.pillar import Pillar
from cvtools.wall import Wall
from cvtools.line import Line
from time import sleep, time
from statistics import median
import cv2
from datetime import datetime
from pathlib import Path
import os
import logging
import threading

GPIO_TRIG_LEFT = 16
GPIO_ECHO_LEFT = 15
GPIO_TRIG_RIGHT = 38
GPIO_ECHO_RIGHT = 37


def change_range(value, in_min, in_max, out_min, out_max):
    scaled_value = (value - in_min) * (out_max - out_min) / \
        (in_max - in_min) + out_min
    return int(scaled_value)


class Car:
    def __init__(self, debug):
        self.cvtool = CVTool(rsp_cam=True)
        self.pillars = (Pillar(self.cvtool, "red"),
                        Pillar(self.cvtool, "green"))
        self.wall = Wall(self.cvtool)
        self.orange_line = Line(self.cvtool, "orange")
        self.blue_line = Line(self.cvtool, "blue")
        self.left_distance_sensor = Distance(
            gpio_trigger=GPIO_TRIG_LEFT, gpio_echo=GPIO_ECHO_LEFT)
        self.right_distance_sensor = Distance(
            gpio_trigger=GPIO_TRIG_RIGHT, gpio_echo=GPIO_ECHO_RIGHT)
        self.ev3 = EV3()
        tries = 0
        while self.ev3.ser is None:
            tries += 1
            print("Fail to connect, try No ", tries)
            self.ev3 = EV3()
            sleep(1)
        self.round_direction = 0
        log_file_path = os.path.dirname(__file__) + "/log_file.log"
        logging.basicConfig(filename=log_file_path,
                            format="%(asctime)s %(message)s",
                            filemode='w')
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)
        self.inner_distance = 0
        self.outer_distance = 0
        self.update_distances = False

    def menu(self):
        self.logger.debug("In menu method")
        if not self.ev3.is_connected():
            self.logger.debug("EV3 is not connected")
            self.ev3 = EV3()
            tries = 0
            while self.ev3.ser is None:
                tries += 1
                print("Fail to connect, try No ", tries)
                self.ev3 = EV3()
                sleep(1)
        self.logger.debug("EV3 is connected")
        self.logger.debug("Waiting command from EV3")
        while True:
            ev3_cmd = self.get_data()
            if ev3_cmd is not None:
                print(ev3_cmd)
                if ev3_cmd[0] == 10:
                    self.logger.debug("Cmd: RUN 1")
                    self.send_data(ev3_cmd)
                    self.run_1()
                elif ev3_cmd[0] == 11:
                    self.logger.debug("Cmd: RUN 2")
                    self.send_data(ev3_cmd)
                    self.run_2(debug=False)
                elif ev3_cmd[0] == 12:
                    self.logger.debug("Cmd: Get Photos")
                    self.send_data(ev3_cmd)
                    self.get_photos()

    def dir_from_lines(self, image, draw=False):
        blue_y = -1
        orange_y = -1
        blue_detection = self.blue_line.detect(image, draw)
        if blue_detection is not None:
            blue_y = blue_detection[1][1]
        orange_detection = self.orange_line.detect(image, draw)
        if orange_detection is not None:
            orange_y = orange_detection[1][1]
        if orange_y == -1 or blue_y == -1:
            return 0
        elif orange_y > blue_y:
            return self.orange_line.turn_side
        else:
            return self.blue_line.turn_side

    def send_data(self, data, open_sign="$"):
        '''
        send data list to ESP32
        '''
        self.ev3.send_data(data=data, open_sign=open_sign)

    def get_data(self, open_sign="$"):
        '''
        get data list from esp32
        '''
        return self.ev3.get_data(open_sign=open_sign)

    def get_photos(self):
        now = datetime.now()
        folder_path = now.strftime("%d_%m_%Y_%H_%M_%S")
        print(folder_path)
        Path(folder_path).mkdir(parents=True, exist_ok=True)
        sleep(1)
        counter = 0
        while True:
            data = self.get_data(open_sign='!')
            if data is not None:
                if data[0] == 1 and self.cvtool.camera_is_open():
                    filename = folder_path + "/img_" + str(counter) + ".png"
                    cv2.imwrite(filename, img)
                    print("Image saved with name ", filename)
                    sleep(0.5)
                    counter += 1
                elif data[0] == 0:
                    break
            _, img = self.cvtool.camera.read()
            cv2.imshow("Image", img)
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def get_photo_packet(self, freq=0.5, duration=180):
        now = datetime.now()
        folder_path = now.strftime("%d_%m_%Y_%H_%M_%S")
        print(folder_path)
        Path(folder_path).mkdir(parents=True, exist_ok=True)
        sleep(1)
        counter = 0
        start_time = time()
        last_time = 0
        current_time = time()
        while current_time - start_time < duration:
            current_time = time()
            _, img = self.cvtool.camera.read()
            if current_time - last_time > freq:
                filename = folder_path + "/img_" + str(counter) + ".png"
                cv2.imwrite(filename, img)
                counter += 1
                last_time = current_time
            cv2.imshow("Image", img)
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    # def run_1(self, rounds=3, wall_distance_threshold=15, k=4):
    #     '''
    #     Messages ([#, ...])
    #     #0 [0]: wait until gets 0 to start
    #     #1 [1, #]: the 2nd number (#) is the round direction (-1 or 1)
    #     #2 [2, #]: the 2nd number is the steering from -100 to 100
    #     #3 [3]: stop the robot
    #     #4 [-4]: change direction to the left
    #     #-4 [4]: change direction to the r_toolight
    #     @5 [5]: turn completed
    #     '''
    #     self.logger.debug("Waiting EV3 to start")
    #     # answer = self.ev3.wait_response(request=0)
    #     # self.ev3.send_data(answer)
    #     self.logger.debug("Message from EV3, GO!")
    #     self.logger.debug("Waiting for round direction ...")
    #     sleep(1)
    #     while self.round_direction == 0:
    #         directions = []
    #         while len(directions) < 7:
    #             if self.cvtool.camera_is_open():
    #                 suc, frame = self.cvtool.camera.read()
    #                 dir = self.dir_from_lines(image=frame, draw=False)
    #                 if dir is not None and dir != 0:
    #                     directions.append(dir)
    #                     self.logger.debug(
    #                         "Dir found: " + str(dir) + ", array: " + str(directions))
    #         median(directions)
    #     self.logger.debug("Round direction: " + str(self.round_direction))
    #     # self.ev3.send_and_confirm(data=[1, self.round_direction])
    #     self.ev3.send_data(data=[1, self.round_direction])
    #     outer_us = self.left_distance_sensor
    #     inner_us = self.right_distance_sensor
    #     if self.round_direction == -1:
    #         outer_us = self.right_distance_sensor
    #         inner_us = self.left_distance_sensor
    #         k = -k
    #     init_distance = outer_us.distance_cm()
    #     turns = 0
    #     last_error = 0
    #     sum_error = 0
    #     last_sample = 0
    #     self.logger.debug("Starting outer wall following")
    #     rounds_passed = 0
    #     last_turn = 0
    #     turn_delay = 3000
    #     target = init_distance
    #     # t = threading.Thread(target=self.t_update_distances, args=(inner_us, outer_us, ))
    #     # t.start()
    #     # sleep(0.05)
    #     last_check = 0
    #     checking_delay = 200
    #     while rounds_passed < rounds:
    #         current_ms = time() * 1000
    #         self.inner_distance = int(inner_us.distance_cm(tries=5))
    #         self.outer_distance = int(outer_us.distance_cm(tries=3))
    #         last_check = current_ms
    #         # inner_distance = int(inner_us.distance_cm(tries=5))
    #         if self.inner_distance > 180 and current_ms - last_turn > turn_delay:
    #             self.ev3.send_data([self.round_direction * 4])
    #             last_turn = current_ms
    #             self.logger.debug("Turn " + str(turns) + ", round " +
    #                               str(rounds_passed) + ", distance " + str(self.inner_distance))
    #             turns += 1
    #             if turns == 4:
    #                 rounds_passed += 1
    #                 turns = 0
    #             answer = self.ev3.wait_response(request=5)
    #         elif current_ms - last_check > checking_delay:
    #             last_check = current_ms
    #             inner_error = self.inner_distance - wall_distance_threshold
    #             # outer_distance = int(outer_us.distance_cm(tries=1))
    #             outer_error = self.outer_distance - wall_distance_threshold
    #             # self.logger.debug("inner_distance:" + str(self.inner_distance) + ", outer_distance:" + str(self.outer_distance) )
    #             # self.logger.debug("inner_distance:" + str(inner_distance) + ", outer_distance:" + str(outer_distance) )
    #             steering = 0
    #             if outer_error < 0:
    #                 steering = -k * outer_error
    #             elif inner_error < 0:
    #                 steering = k * inner_error
    #             if steering != 0:
    #                 self.send_data([2, steering])
    #                 self.logger.debug("Send " + str([2, steering]))
    #     self.update_distances = False

    #     # self.ev3.send_data(data=answer)
    #     self.send_data([3])

    def search_round_direction(self, image):
        dir = self.dir_from_lines(image, draw=False)
        if dir is not None and dir != 0:
            self.round_direction = dir
            self.logger.debug("Dir found: " + str(dir))

    def search_closest_pillar(self, image):
        closer_pillar = None
        for pillar in self.pillars:
            detected_pillar = pillar.detect(image)
            if detected_pillar is not None:
                if closer_pillar is None:
                    closer_pillar = detected_pillar
                elif detected_pillar[1][1] > closer_pillar[1][1]:
                    closer_pillar = detected_pillar
        return closer_pillar

    def run_2(self, cam=True, img_path=None, walls=False, pillars=True, debug=False, l_wall=1.5):
        '''
        Messages ([#, ...])
        #0 [0]: wait until gets 0 to start
        #1 [1, #]: the 2nd number (#) is the round direction (-1 or 1)
        #2 [2, #]: the 2nd number is the steering from -100 to 100
        #3 [3]: stop the robot
        #4 [-4]: change direction to the left
        #-4 [4]: change direction to the right
        @5 [5]: turn complete
        '''
        folder_path = ""
        img_debug_index = 0
        if debug:
            self.logger.debug("Debug mode")
            now = datetime.now()
            folder_path = os.path.dirname(__file__) + now.strftime("/%d_%m_%Y_%H_%M_%S")            
            Path(folder_path).mkdir(parents=True, exist_ok=True)
            self.logger.debug("Folder " + folder_path + " created for the debugging images")

        self.logger.debug("Waiting EV3 to start")
        answer = self.ev3.wait_response(request=0)
        self.logger.debug("Message from EV3, GO!")
        self.logger.debug("Starting outer wall following")
        last_send = 0
        send_delay = 250
        img_i = 1
        # requests:
        # :2 - avoid pillar or side wall
        # :5 - avoid wall in front of the car
        request = 2
        stopped = False
        while True:  # rounds_passed < rounds:
            wall_detected = False
            pillar_detected = False
            line_detected = False
            current_ms = time() * 1000
            # Listen EV3 to stop process
            ev3_msg = self.ev3.get_data()
            if ev3_msg is not None and ev3_msg[0] == 6:
                stopped = True
                sleep(1)
                return None

            # get next frame
            if not cam:
                frame = cv2.imread(f"{img_path}/img_{str(img_i)}.png")
                img_i += 1
            elif self.cvtool.camera_is_open():
                _, frame = self.cvtool.camera.read()

            # init steering
            steering = 0
            # Looking for walls
            wall_image, dir, angle = self.wall.detect_wall(frame)
            if dir != 'N':  # Wall found
                wall_detected = True
                if dir == 'F':
                    # Priority to the walls in front of the car
                    
                    request = 5
                    # steering in this case is always positive
                    steering = 100
                    
                else:
                    request = 2

                    steering = max(-100, min(100, int(angle * l_wall)))
                
                self.logger.debug("Wall detected, steering: " + str(steering) + ", req " + str(request))

            else:
                # No walls, looking for pillars
                closest_pillar = self.search_closest_pillar(image=frame)
                if closest_pillar is not None:
                    pillar_detected = True
                    request = 2
                    steering = closest_pillar[1][0]                
                    # self.logger.debug(
                    #     "Pillar detected, steering: " + str(steering))
                    

            if steering != 0 and current_ms - last_send > send_delay:
                
                last_send = current_ms
                self.send_data([request, steering])
                self.logger.debug("Send " + str([request, steering]))
                if debug and not stopped:
                    
                    if wall_detected:
                        msg = "Wall steering: " + str(steering)
                        img_name = os.path.join(folder_path, "img_" + str(img_debug_index) + ".png")
                        img_debug_index += 1
                        cv2.putText(wall_image, msg, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, 2)
                        cv2.imwrite(img_name, wall_image)
                    elif pillar_detected:
                        msg = "Pillar steering: " + str(steering)
                        img_name = os.path.join(folder_path, "img_" + str(img_debug_index) + ".png")
                        img_debug_index += 1
                        cv2.putText(closest_pillar[0], msg, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, 2)
                        cv2.imwrite(img_name, closest_pillar[0])
            if not cam:
                cv2.imshow("Image", frame)
                cv2.waitKey(0)
            # new_frame_time = time()
            # fps = int(1 / (new_frame_time - prev_frame_time))
            # prev_frame_time = new_frame_time
            # print(fps)

    def run_1(self, cam=True, img_path=None, debug=False, l_wall=1.5):
        '''
        Messages ([#, ...])
        #0 [0]: wait until gets 0 to start
        #1 [1, #]: the 2nd number (#) is the round direction (-1 or 1)
        #2 [2, #]: the 2nd number is the steering from -100 to 100
        #3 [3]: stop the robot
        #4 [-4]: change direction to the left
        #-4 [4]: change direction to the right
        @5 [5]: turn complete
        '''
        folder_path = ""
        img_debug_index = 0
        if debug:
            self.logger.debug("Debug mode")
            now = datetime.now()
            folder_path = os.path.dirname(__file__) + now.strftime("/%d_%m_%Y_%H_%M_%S")            
            Path(folder_path).mkdir(parents=True, exist_ok=True)
            self.logger.debug("Folder " + folder_path + " created for the debugging images")

        self.logger.debug("Waiting EV3 to start")
        answer = self.ev3.wait_response(request=0)
        self.logger.debug("Message from EV3, GO!")
        self.logger.debug("Starting outer wall following")
        last_send = 0
        send_delay = 250
        img_i = 1
        # requests:
        # :2 - avoid pillar or side wall
        # :5 - avoid wall in front of the car
        request = 2
        stopped = False
        while True:  # rounds_passed < rounds:
            current_ms = time() * 1000
            # Listen EV3 to stop process
            ev3_msg = self.ev3.get_data()
            if ev3_msg is not None and ev3_msg[0] == 6:
                stopped = True
                sleep(1)
                return None

            # get next frame
            _, frame = self.cvtool.camera.read()

            # init steering
            steering = 0
            # Looking for walls
            wall_image, dir, angle = self.wall.detect_wall(frame)
            if dir != 'N':  # Wall found
                wall_detected = True
                if dir == 'F':
                    # Priority to the walls in front of the car
                    
                    request = 5
                    # steering in this case is always positive
                    steering = 100
                    
                else:
                    request = 2

                    steering = max(-100, min(100, int(angle * l_wall)))
                
                self.logger.debug("Wall detected, steering: " + str(steering) + ", req " + str(request))


            if steering != 0 and current_ms - last_send > send_delay:
                
                last_send = current_ms
                self.send_data([request, steering])
                self.logger.debug("Send " + str([request, steering]))
             
    
    def run_from_folder(self, walls=False, pillars=False, pid_ks=(1, 0, 0), debug=True, folder_path="19_07_2022_20_30_29"):
        # self.ev3.wait_response(request=0)
        next_img_index = 1
        while self.round_direction == 0:
            directions = []
            while len(directions) < 3:
                image_path = os.path.join(
                    folder_path, "img_" + str(next_img_index) + ".png")
                print("file:", image_path)
                image = cv2.imread(image_path)
                next_img_index = (next_img_index + 1) % 469
                dir = self.dir_from_lines(image=image, draw=False)
                print(dir)
                if dir is not None:
                    directions.append(dir)
            self.round_direction = median(directions)
        print("self.round_direction:", self.round_direction)
        # self.ev3.send_data(data=[1, self.round_direction])
        outter_distance = self.left_distance_sensor
        inner_distance = self.right_distance_sensor
        if self.round_direction == -1:
            outter_distance = self.left_distance_sensor
            inner_distance = self.right_distance_sensor

    # def run_from_folder(self, walls=True, pillars=True, folder_path="19_07_2022_20_30_29"):

    #     steer_angle = 50
    #     # print("EV3 waiting ...")
    #     # self.handshake()
    #     for filename in os.listdir(folder_path):
    #         img = cv2.imread(os.path.join(folder_path,filename))
    #         if img is not None:
    #             if walls:
    #                 wall_steering = 0
    #                 wall_image, dir, steering_angle = self.wall.detect_wall(img)
    #                 print(dir, steering_angle)
    #                 if dir == 'F':
    #                     wall_steering = self.run_direction * 70
    #                 elif dir == 'R' or dir == 'L':
    #                     wall_steering = steering_angle

    #             if pillars:
    #                 pillar_steering = 0
    #                 min_pillar_distance = 150
    #                 closest_pillar = None
    #                 for pillar in self.pillars:
    #                     detected = pillar.detect(img, draw=True)
    #                     if detected and detected[1][0] < min_pillar_distance:
    #                         closest_pillar = detected[:]
    #                         min_pillar_distance = detected[1][0]
    #                 if closest_pillar:
    #                     pillar_steering = closest_pillar[1][1]
    #             if walls and wall_steering != 0:
    #                 steering = wall_steering
    #                 # self.send_data([wall_steering])
    #                 print("Wall steering: ", wall_steering)
    #             elif pillars:
    #                 steering = pillar_steering
    #                 # self.send_data([pillar_steering], open_sign='$')
    #                 print("Pillar steering: ", pillar_steering)
    #             if self.debug:
    #                 if walls and wall_steering != 0:
    #                     cv2.imshow("Video", wall_image)
    #                 elif pillars and closest_pillar is not None:
    #                     cv2.imshow("Video", closest_pillar[0])
    #                 else:
    #                     cv2.imshow("Video", img)
    #                 cv2.waitKey(0)


def main():
    cmd = "sudo chmod 666 /dev/ttyTHS1"
    os.system(cmd)
    car = Car(debug=False)
    car.menu()
    # car.run_2(cam=False, img_path="19_07_2022_20_30_29")
    # car.run()
    # car.run_from_folder(folder_path="19_07_2022_20_30_29")
    # car.get_photos()
    # car.get_photo_packet(duration=240)
    # while True:
    #     car.send_data([1, 10])
    #     sleep(0.5)
    #     print(car.get_data())
    #     sleep(0.5)


if __name__ == "__main__":
    main()
