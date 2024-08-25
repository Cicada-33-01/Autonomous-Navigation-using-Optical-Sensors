
import matplotlib.pyplot as plt
import numpy as np
import serial
import time
import math
import threading
import queue
# Serial communication setup (adjust based on your settings)
serial_port = "com7"
baud_rate = 115200
data_stream = serial.Serial(serial_port, baud_rate)

import math


def getting_data(data_stream):
    # Read and parse accelerometer data (assuming comma-separated values)
    data = data_stream.readline().decode('utf-8')
    if not data:
        return None
    elements = data.split(',')
    # print(elements)
    if len(elements) != 5:
        return (0,0,0,0,0)
    try:
        mag1 , mag2, mag3 , pitch , roll = map(float, elements)
        # acc_z = acc_z + 9.80665
    except ValueError:
        raise ValueError("Invalid data values")
    return mag1 , mag2, mag3 , pitch , roll
# Define state variables (position, velocity, acceleration)




def main():


    # thread4 = threading.Thread(target=Motor_control,args=())
    # thread4.start()
    # thread3 = threading.Thread(target=main_OFS , args=())
    # thread3.start()
    
    while(True):

        DATA_hold = getting_data(data_stream)
        mx = (DATA_hold[0]-127774)*(800/9946)
        my = (DATA_hold[1]-136529)*(800/10616)
        mz = (DATA_hold[2]-134931.5)*(800/9775)

        pitch = DATA_hold[3]
        roll = DATA_hold[4]

        yaw = math.atan2(my * math.cos(roll) - mz * math.sin(roll), mx * math.cos(pitch) + my * math.sin(pitch) * math.sin(roll) + mz * math.cos(roll) * math.sin(pitch))
        print("MX:" ,mx, "  MY:" ,my,"  MZ:",mz , "  yaw:" , yaw)
        # time.sleep(0.05)
main()













