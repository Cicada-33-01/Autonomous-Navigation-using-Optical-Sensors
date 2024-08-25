
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

def compute_displacement(delta_x_L, delta_y_L, delta_x_R, delta_y_R):
    """
    Computes the displacement (delta_x, delta_y, delta_theta) of the robot
    based on the readings from two optical flow sensors (OFS).

    Args:
        delta_x_L (float): Change in x-coordinate from the left OFS
        delta_y_L (float): Change in y-coordinate from the left OFS
        delta_x_R (float): Change in x-coordinate from the right OFS
        delta_y_R (float): Change in y-coordinate from the right OFS

    Returns:
        tuple: (delta_x, delta_y, delta_theta)
               delta_x (float): Change in x-coordinate of the robot
               delta_y (float): Change in y-coordinate of the robot
               delta_theta (float): Change in orientation (in radians) of the robot
    """
    delta_x = (delta_x_L + delta_x_R) / 2
    delta_y = (delta_y_L + delta_y_R) / 2

    L_prime = (delta_x_L, delta_y_L)
    R_prime = (delta_x_R, delta_y_R)
    R_prime_L_prime = (R_prime[0] - L_prime[0], R_prime[1] - L_prime[1])

    delta_theta = math.atan2(R_prime_L_prime[1], R_prime_L_prime[0])

    return delta_x, delta_y, delta_theta

def update_position(x_i, y_i, theta_i, delta_x_i, delta_y_i, delta_theta_i):
    """
    Updates the position of the robot based on the current position and displacement.

    Args:
        x_i (float): Current x-coordinate of the robot
        y_i (float): Current y-coordinate of the robot
        theta_i (float): Current orientation (in radians) of the robot
        delta_x_i (float): Change in x-coordinate of the robot
        delta_y_i (float): Change in y-coordinate of the robot
        delta_theta_i (float): Change in orientation (in radians) of the robot

    Returns:
        tuple: (x_i_plus_1, y_i_plus_1, theta_i_plus_1)
               x_i_plus_1 (float): Updated x-coordinate of the robot
               y_i_plus_1 (float): Updated y-coordinate of the robot
               theta_i_plus_1 (float): Updated orientation (in radians) of the robot
    """
    x_i_plus_1 = x_i + delta_x_i * math.cos(theta_i) - delta_y_i * math.sin(theta_i)
    y_i_plus_1 = y_i + delta_x_i * math.sin(theta_i) + delta_y_i * math.cos(theta_i)
    theta_i_plus_1 = theta_i + delta_theta_i

    return x_i_plus_1, y_i_plus_1, theta_i_plus_1

def getting_data(data_stream):
    # Read and parse accelerometer data (assuming comma-separated values)
    data = data_stream.readline().decode('utf-8')
    if not data:
        return None
    elements = data.split(',')
    # print(elements)
    if len(elements) != 12:
        return (0,0,9.80665,0.0,0.0,0.0,0,0,9.80665,0.0,0.0,0.0)
    try:
        acc_x, acc_y, acc_z,ang_vel_x,ang_vel_y,ang_vel_z,acc_x_2, acc_y_2, acc_z_2,ang_vel_x_2,ang_vel_y_2,ang_vel_z_2 = map(float, elements)
        # acc_z = acc_z + 9.80665
    except ValueError:
        raise ValueError("Invalid data values")
    return acc_x, acc_y, acc_z,ang_vel_x,ang_vel_y,ang_vel_z,acc_x_2, acc_y_2, acc_z_2,ang_vel_x_2,ang_vel_y_2,ang_vel_z_2
# Define state variables (position, velocity, acceleration)

def integrate_angular_velocity(gyro_data, dt, positions):

  data_0 = gyro_data[0]
  data_1 = gyro_data[1]
  data_2 = gyro_data[2]
  # Integrate over time
#   for value in gyro_data:
  if(math.fabs(gyro_data[0])<=0.01):
    data_0 = 0
  positions[0] += data_0 * dt  # Integrate X-axis angular velocity
  if(math.fabs(gyro_data[1])<=0.01):
    data_1 = 0
  positions[1] += data_1 * dt  # Integrate Y-axis angular velocity
  
  if(gyro_data[2]<=0.01):
    data_2 = 0
  positions[2] += data_2 * dt  # Integrate Z-axis angular velocity

  return positions


def fix_orientation_data(acc_x,acc_y,ang_x , ang_y):
    
    theta_1 = ang_y
    theta_2 = ang_x

    acc_x = (math.fabs(acc_x) - 9.85665 * math.sin(theta_1)) / math.cos(theta_1)
    # print(acc_x)
    acc_y = (math.fabs(acc_y) - 9.80665 * math.sin(theta_2)) / math.cos(theta_2)

    return acc_x, acc_y
   

Position_x1 = []
Position_x2 = []

Position_y1 = []
Position_y2 = []
correcting_factor_x1 = (0.0,0.0,0.0,0.0,0.0,0.0)
correcting_factor_x2 = (0.0,0.0,0.0,0.0,0.0,0.0)

correcting_factor_y1 = (0.0,0.0,0.0,0.0,0.0,0.0)
correcting_factor_y2 = (0.0,0.0,0.0,0.0,0.0,0.0)

x1_position_1 = 0
y1_position_2 = 0
x1_velocity_1 = 0
y1_velocity_2 = 0

x1_correcting_factor = (0,0,0,0,0,0)
x1_count =0
x1_start = False


    # start = 0
    # end = 0

    # max_vel = 0
x1_corrected_value_1 = 0
x1_old_value_1 =0

y1_corrected_value_2 = 0
y1old_value_2 =0





x2_position_1 = 0
y2_position_2 = 0
x2_velocity_1 = 0
y2_velocity_2 = 0

x2_correcting_factor = (0,0,0,0,0,0)
x2_count =0
x2_start = False


    # start = 0
    # end = 0

    # max_vel = 0
x2_corrected_value_1 = 0
x2_old_value_1 =0

y2_corrected_value_2 = 0
y2old_value_2 =0


x1_prev_positions = []
y1_prev_positions = []

x2_prev_positions = []
y2_prev_positions = []


#initializations for sir's calibration method
delta_x_L = 0

delta_y_L = 0

delta_x_R = 0

delta_y_R = 0


def iterate(Imu_pos,dir, DATA , dt , correcting_factor , old_value , third_old , position_1 , position_2 , queue_p , velocity_1 , velocity_2): # true = forward anf false = backward
      # Time between measurements

    # print("2")



 
        # if(count%3==0):
        # third_old = old_value_1
        # old_value_1 = corrected_value_1

        # third_old_2 = old_value_2
        # old_value_2 = corrected_value_2
        old_value_1 = old_value[0]
        old_value_2 = old_value[1]

        third_old_1 = third_old[0]
        third_old_2 = third_old[1]
        
        corrected_value_1 = DATA[0] - correcting_factor[0] 
        corrected_value_2 = DATA[1] - correcting_factor[1] 
        # corrected_value_3 = DATA[2] - correcting_factor[2] 
        # print(corrected_value_1)
        # corrected_value_1 = corrected_value_1*0.65+old_value_1*0.3+third_old_1*0.05
        # # print("corr_val:",corrected_value_1)
        # corrected_value_2 = corrected_value_2*0.65+old_value_2*0.3+third_old_2*0.05
        
        if(math.fabs(corrected_value_1)<0.025):
                corrected_value_1 = 0
                correcting_factor = (DATA[0],correcting_factor[1],DATA[2],correcting_factor[3],correcting_factor[4],correcting_factor[5])
                global correcting_factor_x1
                correcting_factor_x1 = correcting_factor
                global x2_velocity_1
                x2_velocity_1 = 0
                velocity_1 =0 
        if(math.fabs(corrected_value_2)<0.025):
                corrected_value_2 = 0
                correcting_factor = (correcting_factor[0],DATA[1],DATA[2],correcting_factor[3],correcting_factor[4],correcting_factor[5])
                global correcting_factor_y1
                correcting_factor_y1 = correcting_factor
                global y2_velocity_2
                y2_velocity_2 = 0
                velocity_2 =0 
        # if(math.fabs(corrected_value_1)>0.05):
        #         corrected_value_1 = 0
        #         correcting_factor = (DATA[0],correcting_factor[1],DATA[2],correcting_factor[3],correcting_factor[4],correcting_factor[5])
        #         global correcting_factor_x1
        #         correcting_factor_x1 = correcting_factor
        #         global x2_velocity_1
        #         x2_velocity_1 = 0
        #         velocity_1 =0 
        # corrected_value_4_ang = (DATA[3] - correcting_factor[3])*20*1.1*3/4
        # corrected_value_5_ang = (DATA[4] - correcting_factor[4])*20*1.1*3/4 
        # corrected_value_6_ang = (DATA[5] - correcting_factor[5])*20*1.1 
        
        
        # gyro_data = (corrected_value_4_ang,corrected_value_5_ang,corrected_value_6_ang)

        # corrected_value = 0

   
        # end = time.time()
        # dt = end - start

        # start = end

        # if(start):
            
            # position_1 += velocity_1 * dt + 0.5 * (corrected_value_1) * dt**2
        # print(dt)
        velocity_1 += corrected_value_1 * dt
        # print("vel:",velocity_1 , "  corr:", corrected_value_1)
        # if(velocity_1>0.3):
        #         max_vel = velocity_1
        if(math.fabs(velocity_1)>=0.03):
                # velocity_1 = 0
                # print("break----------------")
                old_value_1 = corrected_value_1
                third_old = corrected_value_1
                correcting_factor =DATA
        global delta_x_L
        if(dir):
                
                delta_x_L = math.fabs(velocity_1 * dt)

                position_1 += math.fabs(velocity_1 * dt)
        else:
                
                delta_x_L = -math.fabs(velocity_1 * dt)

                position_1 -= math.fabs(velocity_1 * dt)
        # print("pos 1:",position_1*10000 , "  vel:",velocity_1, "  corr",corrected_value_1)



        velocity_2 += corrected_value_2 * dt
        if(math.fabs(velocity_2)>=0.03):
                # velocity_2 = 0
                # velocity_2 = 0
                old_value_2 = corrected_value_2
                third_old = corrected_value_2
                correcting_factor =DATA

            # position_2 += velocity_2 * dt
        global delta_y_L
        if(dir):
                
                delta_y_L = math.fabs(velocity_2 * dt)

                position_2 += math.fabs(velocity_2 * dt)
        else:
                # global delta_y_L
                delta_y_L = -math.fabs(velocity_2 * dt)

                position_2 -= math.fabs(velocity_2 * dt)
        # if(Imu_pos):
        #     Position_x1.append(position_1)
        #     Position_y1.append(position_2)
        # else:
        #     Position_x2.append(position_1)
        #     Position_y2.append(position_2)
        # global x1_position_1
        # global y1_position_2
        # global x1_corrected_value_1
        # global y1_corrected_value_2
        # global x2_velocity_1

        global x2_position_1
        global y2_position_2
        global x2_corrected_value_1
        global y2_corrected_value_2
        global x1_prev_positions
        global y1_prev_positions

        if Imu_pos:
            x2_position_1 = position_1

            # Update the previous positions list
            x1_prev_positions.append(position_1)
            if len(x1_prev_positions) > 3:
                x1_prev_positions.pop(0)  # Remove the oldest value

            # Calculate the average of the previous 3 positions
            if len(x1_prev_positions) == 3:
                x2_position_1 = sum(x1_prev_positions) / 3

            if not queue_p.empty():
                queue_p.put(x2_position_1 * 1000)
            else:
                queue_p.put(x2_position_1)

            y2_position_2 = position_2

            # Update the previous positions list
            y1_prev_positions.append(position_2)
            if len(y1_prev_positions) > 3:
                y1_prev_positions.pop(0)  # Remove the oldest value

            # Calculate the average of the previous 3 positions
            if len(y1_prev_positions) == 3:
                y2_position_2 = sum(y1_prev_positions) / 3

            x2_corrected_value_1 = corrected_value_1
            y2_corrected_value_2 = corrected_value_2


        # else:
        #     x2_position_1 = position_1
        #     # print("why")
        #     if not queue_p.empty():
        #         # hold = queue_p.get_nowait()
        #         # print("hold:",hold*1000)
        #         hold = position_1
        #         # print("hold2:",hold*1000)
        #         queue_p.put(hold)
        #     else:
        #         queue_p.put(position_1)
        #     # print(x1_position_1)
        #     y2_position_2 = position_2
        #     x2_corrected_value_1 = corrected_value_1
        #     y2_corrected_value_2 = corrected_value_2












def iterate2(Imu_pos,dir, DATA , dt , correcting_factor , old_value , third_old , position_1 , position_2 , queue_p , velocity_1 , velocity_2): # true = forward anf false = backward
      # Time between measurements

    # print("2")



 
        # if(count%3==0):
        # third_old = old_value_1
        # old_value_1 = corrected_value_1

        # third_old_2 = old_value_2
        # old_value_2 = corrected_value_2
        old_value_1 = old_value[0]
        old_value_2 = old_value[1]

        third_old_1 = third_old[0]
        third_old_2 = third_old[1]
        
        corrected_value_1 = DATA[0] - correcting_factor[0] 
        corrected_value_2 = DATA[1] - correcting_factor[1] 
        # corrected_value_3 = DATA[2] - correcting_factor[2] 
        # print(corrected_value_1)
        # corrected_value_1 = corrected_value_1*0.65+old_value_1*0.3+third_old_1*0.05
        # corrected_value_2 = corrected_value_2*0.65+old_value_2*0.3+third_old_2*0.05
        
        if(math.fabs(corrected_value_1)<0.025):
                corrected_value_1 = 0
                correcting_factor = (DATA[0],correcting_factor[1],DATA[2],correcting_factor[3],correcting_factor[4],correcting_factor[5])
                global correcting_factor_x2
                correcting_factor_x2 = correcting_factor
                global x1_velocity_1
                x1_velocity_1 = 0
                velocity_1 =0 
        if(math.fabs(corrected_value_2)<0.025):
                corrected_value_2 = 0
                correcting_factor = (correcting_factor[0],DATA[1],DATA[2],correcting_factor[3],correcting_factor[4],correcting_factor[5])
                global correcting_factor_y2
                correcting_factor_y2 = correcting_factor
                global y1_velocity_2
                y1_velocity_2 = 0
                velocity_2 =0 
        
        # corrected_value_4_ang = (DATA[3] - correcting_factor[3])*20*1.1*3/4
        # corrected_value_5_ang = (DATA[4] - correcting_factor[4])*20*1.1*3/4 
        # corrected_value_6_ang = (DATA[5] - correcting_factor[5])*20*1.1 
        
        
        # gyro_data = (corrected_value_4_ang,corrected_value_5_ang,corrected_value_6_ang)

        # corrected_value = 0

   
        # end = time.time()
        # dt = end - start

        # start = end

        # if(start):
            
            # position_1 += velocity_1 * dt + 0.5 * (corrected_value_1) * dt**2
        velocity_1 += corrected_value_1 * dt
        # if(velocity_1>0.3):
        #         max_vel = velocity_1
        if(math.fabs(velocity_1)>=0.03):
                # velocity_1 = 0
                # print("break----------------")
                old_value_1 = corrected_value_1
                third_old = corrected_value_1
                correcting_factor =DATA
        global delta_x_R
        if(dir):
                
                delta_x_R = math.fabs(velocity_1 * dt)
                position_1 += math.fabs(velocity_1 * dt)
        else:
                # global delta_x_R
                delta_x_R = math.fabs(velocity_1 * dt)
                position_1 -= math.fabs(velocity_1 * dt)
        # print("pos :",position_1*1000)



        velocity_2 += corrected_value_2 * dt
        if(math.fabs(velocity_2)>=0.03):
                # velocity_2 = 0
                # velocity_2 = 0
                old_value_2 = corrected_value_2
                third_old = corrected_value_2
                correcting_factor =DATA

            # position_2 += velocity_2 * dt
        global delta_y_R
        if(dir):
                
                delta_y_R = math.fabs(velocity_2 * dt)
                position_2 += math.fabs(velocity_2 * dt)
        else:
                # global delta_y_R
                delta_y_R = math.fabs(velocity_2 * dt)
                position_2 -= math.fabs(velocity_2 * dt)
        # if(Imu_pos):
        #     Position_x1.append(position_1)
        #     Position_y1.append(position_2)
        # else:
        #     Position_x2.append(position_1)
        #     Position_y2.append(position_2)
        global x1_position_1
        global y1_position_2
        global x2_prev_positions
        global y2_prev_positions
        global x1_corrected_value_1
        global y1_corrected_value_2


        if(Imu_pos):
            print("NO WAY")
            # x1_position_1 = position_1
            # if not queue_p.empty():
            #     # hold = queue_p.get()
            #     # print("hold:",hold*1000)
            #     hold = position_1
            #     # print("pos:",position_1)
            #     queue_p.put(hold*1000)
            # else:
            #     queue_p.put(position_1)
            # # print(x1_position_1)
            # y1_position_2 += position_2
            # x1_velocity_1 = velocity_1
            # x1_corrected_value_1 = corrected_value_1
            # y1_corrected_value_2 = corrected_value_2
        else:
            x1_position_1 = position_1

            # Update the previous positions list
            x2_prev_positions.append(position_1)
            if len(x2_prev_positions) > 3:
                x2_prev_positions.pop(0)  # Remove the oldest value

            # Calculate the average of the previous 3 positions
            if len(x2_prev_positions) == 3:
                x1_position_1 = sum(x2_prev_positions) / 3

            if not queue_p.empty():
                queue_p.put(x1_position_1 * 1000)
            else:
                queue_p.put(x1_position_1)

            y1_position_2 = position_2

            # Update the previous positions list
            y2_prev_positions.append(position_2)
            if len(y2_prev_positions) > 3:
                y2_prev_positions.pop(0)  # Remove the oldest value

            # Calculate the average of the previous 3 positions
            if len(y2_prev_positions) == 3:
                y1_position_2 = sum(y2_prev_positions) / 3

            x1_corrected_value_1 = corrected_value_1
            y1_corrected_value_2 = corrected_value_2
        
        # return position_1,position_2,corrected_value_1,corrected_value_2
            # data_queue.put(position_1)
    

    # plt.plot(True_data, label='True disp')
    #     # plt.plot(gain, label='Gain')
    # plt.plot(measured_data, label='Measured Displacement')  # Label for measured data

    # plt.legend()
    # plt.xlabel("Time Step")
    # plt.ylabel("Displacement (X-axis)")
    # plt.title("Measured vs. Filtered Displacement Over Time")  # More informative title

    #     # Show the plot
    # plt.show()
# Constants













import struct
import time
import math
import evdev
from evdev import InputDevice


# Ensure each mouse references a different device
#mouse1 = InputDevice('/dev/input/event5')  # Assuming first mouse is at /dev/input/event0
#mouse2 = InputDevice('/dev/input/event8')  # Assuming second mouse is at /dev/input/event1


def find_left_mouse(event_numbers):
    for i in event_numbers:
        try:
            device = InputDevice(f'/dev/input/event{i}')
            if "mouse" in device.name.lower():
                return device
        except:
            pass
    raise Exception('Left mouse not found')

def find_right_mouse(event_numbers):
    for i in event_numbers:
        try:
            device = InputDevice(f'/dev/input/event{i}')
            if "mouse" in device.name.lower():
                return device
        except:
            pass
    raise Exception('Right mouse not found')

mouse1 = find_left_mouse(range(5, 8))  # Check events 5 to 7 for left mouse
mouse2 = find_right_mouse(range(8, 11))  # Check events 8 to 10 for right mouse

dpi1 = 16000  # DPI of mouse1
dpi2 = 1700  # DPI of mouse2

def get_delta(mouse):
    dx, dy = 0, 0
    while True:
        r, w, x = select.select([mouse], [], [], 0.01)
        if r:
            for event in mouse.read():
                if event.type == evdev.ecodes.EV_REL:
                    if event.code == evdev.ecodes.REL_X:
                        dx = event.value
                    elif event.code == evdev.ecodes.REL_Y:
                        dy = event.value
                        return dx, dy
        else:
            return dx, dy

x, y = 0, 0  # Initial position
t1 = time.time()

X,Y = x,y
x_positions_OFS = [0]
y_positions_OFS = [0]
orientations_OFS = [0]

def main_OFS():
        # def output_position_ofs(x, y):
        print(x,y)
        # Create a figure and axis for the plot
        fig, ax = plt.subplots()
        # Create a scatter plot for the bot's position
        scat = ax.scatter([], [], s=100)
        # Create a line plot for the bot's path
        path, = ax.plot([], [], 'r-')

        # Enable grid
        ax.grid(True)

        # Lists to store all positions
        x_positions_OFS = [x]
        y_positions_OFS = [y]

        while True:
            start_time  =   time.time()
            dx1, dy1 = get_delta(mouse1)
            dx2, dy2 = get_delta(mouse2)

            t2 = time.time()

            # Integrate position
            x += ((dx1/dpi1 + dx2/dpi2)/2)
            y += ((dy1/dpi1 + dy2/dpi2)/2)

            # If the dot exits the graph, break the loop
            if x < -100 or x > 100 or y < -100 or y > 100:
                break

            # Add new position to lists
            x_positions_OFS.append(x)
            y_positions_OFS.append(y)

            # Update scatter plot and path plot
            scat.set_offsets([x, y])
            path.set_data(x_positions_OFS, y_positions_OFS)

            # Redraw the plot
            plt.draw()
            plt.pause(0.01)

            # Calculate orientation
            orientation = math.atan2((dy1/dpi1 + dy2/dpi2)/2, (dx1/dpi1 + dx2/dpi2)/2)

            orientations_OFS[0] = orientation
            # Print estimated position
            print('Estimated Position: ({}, {}), estimated orientaion from the origin: ({})'.format(x, y, orientation))
    
            # Update time
            t1 = t2
            end_time = time.time()  # Capture the end time
            # Calculate and print the time taken for this iteration
            iteration_time = end_time - start_time
            print('Time taken for this iteration: {} seconds'.format(iteration_time))
 







import RPi.GPIO as GPIO
import keyboard
from time import sleep

# Pins for Motor Driver Inputs
Motor1A = 16
Motor1B = 20
Motor2A = 21
Motor2B = 26  # Assuming these are the GPIO pins for the second motor
#Motor2B = 17
#Motor2E = 25
 

 
def setup():
    GPIO.setmode(GPIO.BCM)             # GPIO Numbering
    GPIO.setup(Motor1A, GPIO.OUT)      # All pins as Outputs
    GPIO.setup(Motor1B, GPIO.OUT)
    #GPIO.setup(Motor1E, GPIO.OUT)
    GPIO.setup(Motor2A, GPIO.OUT)
    GPIO.setup(Motor2B, GPIO.OUT)
    #GPIO.setup(Motor2E, GPIO.OUT)
    global motor1_pwm

def move_forward(speed):
   # motor1_pwm.ChangeDutyCycle(speed)
    print('hello')
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    #GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    #GPIO.output(Motor2E, GPIO.HIGH)(
    sleep(1)

def move_backward():
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    #GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    #GPIO.output(Motor2E, GPIO.HIGH)
    sleep(1)

def move_left():
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    #GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    #GPIO.output(Motor2E, GPIO.HIGH)
    sleep(1)

def move_right():
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    #GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    #GPIO.output(Motor2E, GPIO.HIGH)
    sleep(1)

def stop():
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor2A, GPIO.LOW)


def Motor_control():
    setup()
    try:
        while True:
            #move_forward(10)
            direction = input("Enter direction (w: forward, s: backward, any other key to stop): ")
            if direction == 'w':
                print('W')
                move_forward(10)
            elif direction == 's':
                print('S')
                move_backward()
            elif direction == 'a':
                print('A')
                move_left()
            elif direction == 'd':
                print('D')
                move_right()
            else:
                stop()
    except KeyboardInterrupt:
        stop()
     

def main():

    queue1 = queue.Queue()
    queue2 = queue.Queue()
    x1_old_value = [0,0]
    x2_old_value = [0,0]
    x1_third_old = [0,0]
    x2_third_old = [0,0]

    global x1_velocity_1
    global y1_velocity_2 
    global x2_velocity_1 
    global y2_velocity_2 
    global correcting_factor_x1
    global correcting_factor_x2

    dt = 0.11

    # Wait for threads to finish
    # thread1.join()
    # thread2.join()
    # print("end")
    # print(len(Position_x1))
    # print(len(Position_x2))

    x_i = 0.0
    y_i = 0.0
    theta_i = 0.0



    start=0
    end = 0
    count =0
    count_dt = 0
    dtFlag = True
    tot_dt =0
    thread4 = threading.Thread(target=Motor_control,args=())
    thread4.start()
    thread3 = threading.Thread(target=main_OFS , args=())
    thread3.start()
    
    while(True):

        if(count<=20):
            # print("haha")

            DATA_hold = getting_data(data_stream)
            DATA1 = (DATA_hold[0],DATA_hold[1],DATA_hold[2],DATA_hold[3],DATA_hold[4],DATA_hold[5])
            DATA2 = (DATA_hold[6],DATA_hold[7],DATA_hold[8],DATA_hold[9],DATA_hold[10],DATA_hold[11])
            correcting_factor_x1 = DATA1
            correcting_factor_x2 = DATA2
            # print("lol")
            start = True
            count+=1
            continue
        if(count==400):
            break
        # print("start")
        count+=1
        # if():
        DATA_hold = getting_data(data_stream)
        DATA1 = (DATA_hold[0],DATA_hold[1],DATA_hold[2],DATA_hold[3],DATA_hold[4],DATA_hold[5])
        # print(DATA1)
        # else:
        #     DATA_hold = getting_data(data_stream)
        DATA2 = (DATA_hold[6],DATA_hold[7],DATA_hold[8],DATA_hold[9],DATA_hold[10],DATA_hold[11])

        # correcting_factor_x1 = DATA1
        # correcting_factor_x2 = DATA2

        x1_corrected_value_1 = DATA1[0] - correcting_factor_x1[0]
        y1_corrected_value_2 = DATA1[1] - correcting_factor_x1[1]
        x1_third_old[0] = x1_old_value[0]
        x1_old_value[0] = x1_corrected_value_1

        x1_third_old[1] = x1_old_value[1]
        x1_old_value[1] = y1_corrected_value_2

        dir = [True,True]
        if(True):
            if(True):
                dir[0] = True
            else:
                dir[0] = False
        else:
            if(Motor_right_moving):
                dir[0] = True
            else:
                dir[0] = False
        # if(count%3==0):
        # thread1 = threading.Thread(target=iterate , args=(True,True,True,))
        # thread2 = threading.Thread(target=iterate , args=(True,True,False,))
    
        # DATA_hold = getting_data(data_stream)
        # DATA1 = (DATA_hold[0],DATA_hold[1],DATA_hold[2],DATA_hold[3],DATA_hold[4],DATA_hold[5])
        # # else:
        # #     DATA_hold = getting_data(data_stream)
        # DATA2 = (DATA_hold[6],DATA_hold[7],DATA_hold[8],DATA_hold[9],DATA_hold[10],DATA_hold[11])

        # correcting_factor_x1 = DATA1
        # correcting_factor_x2 = DATA2

        x2_corrected_value_1 = DATA2[0] - correcting_factor_x2[0]
        # print(x2_corrected_value_1)
        y2_corrected_value_2 = DATA2[1] - correcting_factor_x2[1]
        x2_third_old[0] = x2_old_value[0]
        x2_old_value[0] = x2_corrected_value_1

        x2_third_old[1] = x2_old_value[1]
        x2_old_value[1] = y2_corrected_value_2

        # dir = [True,True]
        if(False):
            if(True):
                dir[1] = True
            else:
                dir[1] = False
        else:
            if(True):
                dir[1] = True
            else:
                dir[1] = False

        


        # x1_velocity_1 += x1_corrected_value_1 * dt
        # # print ("vel1:",x1_velocity_1)
        # x2_velocity_1 += x2_corrected_value_1 * dt

        # y1_velocity_2 += y1_corrected_value_2 * dt
        # y2_velocity_2 += y2_corrected_value_2 * dt

        

        thread1 = threading.Thread(target=iterate, args = (True, dir[0] , DATA1 , dt , correcting_factor_x1 , x1_old_value , x1_third_old , x2_position_1 , y2_position_2, queue1, x2_velocity_1 , y2_velocity_2) )
        thread2 = threading.Thread(target=iterate2, args = (False, dir[1] , DATA2 , dt , correcting_factor_x2 , x2_old_value , x2_third_old , x1_position_1 , y1_position_2 , queue2, x1_velocity_1 , y1_velocity_2) )
        
        
        thread1.start()
        thread2.start()
        
        
        # end = time.time()
        # dt = end - start
        # if(dtFlag):
            
        #     dtFlag = False
        # else:
        #      tot_dt += dt
        #      count_dt+=1
        #      print("average dt:",tot_dt/count_dt)
        #     # print("time:",dt)
        # start = end
        
        dt = 0.1107
        
        # for i in range(5):
                

        

        thread1.join()
        thread2.join()
        # corrected_value_1 = x1_corrected_value_1
        # corrected_value_2 = y1_corrected_value_2        
        x1_old_value[0] = x1_corrected_value_1
        x1_old_value[1] = y1_corrected_value_2

        # corrected_value_1 = x1_corrected_value_1
        # corrected_value_2 = y1_corrected_value_2        
        x2_old_value[0] = x2_corrected_value_1
        x2_old_value[1] = y2_corrected_value_2



        # iterate() 

        delta_x_i, delta_y_i, delta_theta_i = compute_displacement(delta_x_L, delta_y_L, delta_x_R, delta_y_R)

        x_i_plus_1, y_i_plus_1, theta_i_plus_1 = update_position(x_i, y_i, theta_i, delta_x_i, delta_y_i, delta_theta_i)
        

        x_i = x_i_plus_1
        y_i = y_i_plus_1

        theta_i = theta_i_plus_1

        print("X1 : ",x1_position_1,"   X2 :",x2_position_1 , "    Y1 : ",y1_position_2 , "   Y2:",y2_position_2)
        # print("X1 : ",x1_velocity_1,"   X2 :",x2_velocity_1,"  Y1:",y1_velocity_2 , "    y2", y2_velocity_2)
        # print(theta_i)
        # if(Imu_pos):
        #     Position_x1.append(position_1)
        #     Position_y1.append(position_2)
        # else:
        #     Position_x2.append(position_1)
        #     Position_y2.append(position_2)
        # val1 = queue1.get_nowait()
        # print("a",val1)
        # queue1.put(val1)
        # val2 = queue2.get_nowait()
        # print("a",val2)
        # queue2.put(val2)
    thread4.join()
    thread3.join()
    
main()













