"""
Serial Communication Module
Example Send: $0000#0027\n

Example Receive: $0123#0456\n

-Install
sudo apt-get install python3-serial
sudo usermod -a -G tty robotonio
-Allow read and write access to port
sudo chmod 666 /dev/ttyTHS1

"""

from multiprocessing.connection import answer_challenge
import serial
from time import time, sleep
from random import randrange

def zfl(s, width):
    # Pads the provided string with leading 0's to suit the specified 'chrs' length
    # Force # characters, fill with leading 0's
    return '{:0>{w}}'.format(s, w=width)

class EV3:
    def __init__(self, portNo="/dev/ttyTHS1", baudRate=115200):
        """
        :param portNo: COM port No e.g.
            Linux: '/dev/ttyTHS1'
        :param baudRate: BaudRate set in the Serial Device
        :return: Initialzed Serial Object
        """
        #subprocess.run(["perm.txt"])
        try:
            self.ser = serial.Serial(
                port=portNo,
                baudrate=baudRate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            print("Device Connected ")
        except:
            print("Not Connected ")
    
    def send_data(self, data, open_sign='$', close_sign='^'):
        """
        :param data: Data List that needs to be transmitted, 
                    eg [50, 45]: steer=50% & speed=-45%
        :param digits: digits per value  
                    eg digits = 4: message = "$0050-045"  
        :return: null
        """

        myString = open_sign
        for d in data:
            # d == 50 -> "$50#"
            # d == 60 -> "$50#60#"
            myString += str(d) + "#"
        myString = myString[:-1] # remove last '#'
        myString += close_sign
        try:
            self.ser.write(myString.encode())
            # print("send: ", myString)
        except:
            print("Data Transmission Failed ")

    def send_and_confirm(self, data, open_sign="$"):
        code = randrange(0, 1000)
        if len(data) > 1:
            data.insert(1, code)
        else:
            data.append(code)
        self.send_data(data, open_sign)
        sleep(0.005)
        answer = self.get_data(open_sign)
        while answer is None or answer[1] != code:
            self.send_data(data, open_sign)
            sleep(0.005)
            answer = self.get_data(open_sign)

    def get_and_confirm(self, open_sign='$'):
        data = self.get_data(open_sign)
        if data is not None:
            self.send_data(data, open_sign)
        return data

    def get_data(self, open_sign='$'):
        """
        :return: list of data received
        """
        if self.ser.inWaiting() > 0:
            data = self.ser.readline()
            # print("Incoming data:", data)
            data = data.decode("utf-8")
            if data[0] == open_sign:
                data = data[1:]
                data = data.split('#')
                dataList = []
                for d in data:
                    try:
                        i = int(d)
                        dataList.append(i)
                    except ValueError:
                        print("Value error in incoming data")
                        return None
                return dataList
        return None

    def wait_response(self, request=0):
        answer = self.get_data()
        while answer is None or answer[0] != request:
            sleep(0.005)
            answer = self.get_data()
        # sleep(0.05)
        # self.send_data(answer)
        # print(answer)
        return answer

    def is_connected(self):
        return self.ser is not None 

def main():
    ev3 = EV3()
    last_send = 0
    time_to_send = 2#randrange(0,2) 
    while True:
        current_time = time()
        dt = current_time - last_send
        data = ev3.get_data()  # Get List of ESP32 Value
        # print(data)
        if data is not None:
            print("received:", data)    # Print First value.
        if dt > time_to_send:
            ev3.send_data([0, 255]) # Send the Value back\
            last_send = current_time
            #time_to_send = randrange(0, 2)
        
        

if __name__ == "__main__":
    # Connect to Arduino Device
    main()