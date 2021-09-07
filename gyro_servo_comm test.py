# 1) user set servo position in python
# 2) position is sent to arduino
# 3) arduino moves servo to position
# 4) arduino send confirmation message back to python
# 5) message is printed in python console

import serial  # import serial library
import time

dx = 20
command = ""
pos = 0

arduino = serial.Serial('COM7', 115200, timeout=1)  # create serial object named arduino
while True:  # create loop


    # pos += dx
    #
    # if pos > 179 or pos < 1:
    #     dx *= (-1)
    #
    # command = str(pos)
    #
    # arduino.write(command.encode())

    #time.sleep(1)
    reachedPos = str(arduino.readline())  # read serial port for arduino echo
    print(reachedPos)  # print arduino echo to console

