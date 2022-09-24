from pybricks.tools import wait
from random import randrange

class Jetson:
    def __init__(self, ev3, serial):
        self.ev3 = ev3
        self.serial = serial

    def send_data(self, data, open_sign="$"):
        my_string = open_sign
        for d in data:
            my_string += str(d) + "#"
        my_string = my_string[:-1] # remove last '#'
        my_string += "\n"
        try:
            self.serial.write(my_string.encode())
            # print("sent:", my_string.encode())
        except:
            print("Data Transmission Failed")
            self.ev3.screen.clear()
            self.ev3.screen.print("Transmission Failed")
            self.ev3.screen.print(my_string)

    def send_and_confirm(self, data, open_sign="$"):
        code = randrange(0, 1000)
        if len(data) > 1:
            data.insert(1, code)
        else:
            data.append(code)
        self.send_data(data, open_sign)
        wait(5)
        answer = self.get_data(open_sign="$")
        while answer is None or answer[1] != code:
            self.send_data(data, open_sign)
            wait(5)
            answer = self.get_data(open_sign="$")
        
            
    def get_and_confirm(open_sign="$"):
        data = self.get_data(open_sign)
        if data is not None:
            self.send_data(data, open_sign)
        return data

    def get_data(self, open_sign="$", close_sign="^"):
        if self.serial.waiting() > 0:
            try:
                read_char = self.serial.read(length=1).decode()
            except UnicodeError:
                print("Value error")
                return None
            if read_char != open_sign:
                return None
            str_data = ""
            while read_char != close_sign:
                try:
                    read_char = self.serial.read(length=1).decode()
                    str_data += read_char
                except UnicodeError:
                    print("Value error")
                    return None
            str_data = str_data[:-1]
            data = str_data.split('#')
            data_list = []
            for d in data:
                try:
                    i = int(d)
                    data_list.append(int(d))
                except ValueError:
                    print("Value Error in Incoming Data: ", str_data, "___", d)
                    # print(str(str_data))
                    return None
            # print(data_list)
            return data_list 
        # else:
        #     self.ev3.screen.print("No data waiting")
        return None

    def wait_response(self, request=0):
        answer = self.get_data()
        while answer is None or answer[0] != request:
            wait(5)
            answer = self.get_data() 
        print("Answer:", answer)
        # self.ev3.screen.clear()
        # self.ev3.screen.print("Answer")
        # self.ev3.screen.print(str(answer))
        self.send_data(answer)
        return answer
        

    def listen(self):
        while True:
            self.ev3.screen.clear()
            data = self.get_data(open_sign='$')
            self.ev3.screen.print(str(data))
            wait(100)

    def speak(self):
        while True:
            self.ev3.screen.clear()
            self.send_data([11], open_sign='$')
            # self.ev3.screen.print(str(data))
            wait(100)

