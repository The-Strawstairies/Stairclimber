import serial
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.optimize import curve_fit
from matplotlib import style
import threading
import queue
import time
import sys, select, termios, tty


def setup_serial():
    """ Setup serial connection object"""
    arduino_com_port = "/dev/tty.usbmodem14201"
    baud_rate = 115200
    serial_port = serial.Serial(arduino_com_port, baud_rate, timeout=1)
    return serial_port


def parse_line(line_of_data, recorder):
    """Handle input from the serial monitor and logs it. Features the following codes
    (with examples)
    E.G: ['LOG', 'TIME', '500', Motors', '25.25', '24.75']"""

    # split output by commas
    output = [str(x) for x in line_of_data.replace("\r\n", "").split(',')]

    # print(output)
    if output:
        if output[0] == "LOG":
            # handle logged data
            m_l = output[2]
            m_r = output[3]
            t = output[5]
            print("Motor Outputs: (L)", m_l, "(R)", m_r)
            new_items = [t, m_l, m_r]

            recorder.append(new_items)
    return recorder


def getKey():
    """Get a keyboard input without requiring a new line too be entered
    TODO: fix spacing issues present in output"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def read_kbd(input_queue):
    """read from the keyboard and put it into the cross-thread queue
    to later run actions on the robot with the laptop keyboard.
    input_queue: the queue we are going to add presses to"""
    while True:
        # reading = input()
        key = getKey()
        input_queue.put(key)


def main():
    """main program loop"""
    input_reading = ""
    # create a new queue and thread,
    # assigning the thread to run the keyboard reading function
    # as a daemon
    input_queue = queue.Queue()
    kbd_thread = threading.Thread(target=read_kbd,
                                  args=(input_queue,),
                                  daemon=True)
    kbd_thread.start()

    # setup the serial object
    serial_handler = setup_serial()

    # headers for our soon-to-be data frame
    # data_recorder = [["t", "ml", "mr"]]

    while True:

        # log the data and run parsing function
        # line_of_data = serial_handler.readline().decode()
        # data_recorder = parse_line(line_of_data, data_recorder)

        # https://stackoverflow.com/questions/5404068/how-to-read-keyboard-input/53344690#53344690
        # for getting non blocking threaed kbd input. Used for figuring out
        # queues and threading usage
        if input_queue.qsize() > 0:
            input_reading = input_queue.get()
            if (input_reading[0] == "S" or input_reading[0] == "s"):
                serial_handler.write("S".encode())
                # print("python side start\n")
                # reset the nested list system when we start a new run
                # data_recorder = [["t", "ml", "mr"]]
            elif (input_reading[0] == "E" or input_reading[0] == "e"):
                serial_handler.write("E".encode())
                # print("python side end\n")
                # convert to dataframe and write to a csv when
                # we stop the program!
                # df = pd.DataFrame(data_recorder[1:], columns=data_recorder[0])
                # df.to_csv("robot_run", sep=',', encoding='utf-8')
                # print it out to see if that worked
                # print(df)
            elif (input_reading[0] == "V" or input_reading[0] == "v"):
                # change the speed of the robot
                output = "V" + input_reading[1:]
                # serial_handler.write(output.encode())
                serial_handler.write("V75".encode())
                # print("python side speed change\n")
            # Keyboard Movement
            elif (input_reading[0] == "T" or input_reading[0] == "t"):
                # Forward
                serial_handler.write("T".encode())
                # print("python side go forwards\n")
            elif (input_reading[0] == "F" or input_reading[0] == "f"):
                # Left
                serial_handler.write("F".encode())
                # print("python side left turn\n")
            elif (input_reading[0] == "G" or input_reading[0] == 'g'):
                # Backwards
                serial_handler.write("G".encode())
                # print("python side backwards\n")
            elif (input_reading[0] == "H" or input_reading[0] == "h"):
                # Right
                serial_handler.write("H".encode())
                # print("python side right turn\n")
            elif (input_reading[0] == "R" or input_reading[0] == "r"):
                # stop
                serial_handler.write("R".encode())
                # print("python side stop\n")
            elif (input_reading[0] == "K" or input_reading[0] == "k"):
                # stop
                break


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()
