import struct
import time
import math
import evdev
from evdev import InputDevice
import select
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

x_positions = [0]
y_positions = [0]
orientations = [0]

#def output_position_ofs(x, y):
#    fig, ax = plt.subplots()
#    scat = ax.scatter([], [], s=100)
#
#    def update(frame):
#        nonlocal x, y
#        start_time = time.time()
#        dx1, dy1 = get_delta(mouse1)
#        dx2, dy2 = get_delta(mouse2)
#        t2 = time.time()
#        x += ((dx1/dpi1 + dx2/dpi2)/2)
#        y += ((dy1/dpi1 + dy2/dpi2)/2)
#        x_positions[0] = x
#        y_positions[0] = y
#        scat.set_offsets([x, y])
#        orientation = math.atan2((dy1/dpi1 + dy2/dpi2)/2, (dx1/dpi1 + dx2/dpi2)/2)
#        orientations[0] = orientation
#        print('Estimated Position: ({}, {}), estimated orientaion from the origin: ({})'.format(x, y, orientation))
#        t1 = t2
#        end_time = time.time()
#        iteration_time = end_time - start_time
#        print('Time taken for this iteration: {} seconds'.format(iteration_time))
# def output_position_ofs(x, y):

#     fig, ax = plt.subplots()

#     scat = ax.scatter([], [], s=100)

#     line, = ax.plot([], [], 'r')  # Line for the trail


#    ani = FuncAnimation(fig, update, interval=200)
#    plt.show()



def output_position_ofs(x, y):
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
    x_positions = [x]
    y_positions = [y]

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
        x_positions.append(x)
        y_positions.append(y)

        # Update scatter plot and path plot
        scat.set_offsets([x, y])
        path.set_data(x_positions, y_positions)

        # Redraw the plot
        plt.draw()
        plt.pause(0.01)

        # Calculate orientation
        orientation = math.atan2((dy1/dpi1 + dy2/dpi2)/2, (dx1/dpi1 + dx2/dpi2)/2)

        orientations[0] = orientation
        # Print estimated position
        print('Estimated Position: ({}, {}), estimated orientaion from the origin: ({})'.format(x, y, orientation))
 
        # Update time
        t1 = t2
        end_time = time.time()  # Capture the end time
        # Calculate and print the time taken for this iteration
        iteration_time = end_time - start_time
        print('Time taken for this iteration: {} seconds'.format(iteration_time))


output_position_ofs(0, 0)