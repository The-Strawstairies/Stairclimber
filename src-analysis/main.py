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


def setup_serial():
    """ Setup serial connection object"""
    arduino_com_port = "/dev/tty.usbmodem14201"
    baud_rate = 115200
    serial_port = serial.Serial(arduino_com_port, baud_rate, timeout=1)
    return serial_port


def parse_line(line_of_data, recorder):
    """Handle input from the serial monitor and logs it. Features the following codes
    (with examples)
    START code -> S
    STOP code  -> E
    P code     -> P1.0
    I code     -> I0.0
    D code     -> D0.0
    line_of_data: unplit string of line output from serial output
    E.G: ['LOG', 'Motors', '25.25', '24.75', 'Sensors', '956.00', '951.00', 'PID', '1.00', '0.00', '0.00']"""

    # split output by commas
    output = [str(x) for x in line_of_data.replace("\r\n", "").split(',')]

    # print(output)
    if output:
        if output[0] == "LOG":
            # handle logged data
            m_l = output[2]
            m_r = output[3]
            s_l = output[5]
            s_r = output[6]
            p_c = output[8]
            i_c = output[9]
            d_c = output[10]
            run_state = output[11]
            t = output[12]
            print("Motor Outputs: (L)", m_l, "(R)", m_r,
                  "Sensor Outputs: (L)", s_l, "(R)", s_r,
                  "PID Constants: (P)", p_c, "(I)", i_c, "(D)", d_c,
                  "run", run_state)
            new_items = [t, m_l, m_r, s_l, s_r, p_c, i_c, d_c]

            recorder.append(new_items)
    return recorder


def read_kbd(input_queue):
    """read from the keyboard and put it into the cross-thread queue
    to later run actions on the robot with the laptop keyboard."""
    while True:
        reading = input()
        input_queue.put(reading)


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
    data_recorder = [["t", "ml", "mr", "sl", "sr", "p", "i", "d"]]

    while True:

        # log the data and run parsing function
        line_of_data = serial_handler.readline().decode()
        data_recorder = parse_line(line_of_data, data_recorder)

        # https://stackoverflow.com/questions/5404068/how-to-read-keyboard-input/53344690#53344690
        # for getting non blocking threaed kbd input. Used for figuring out
        # queues and threading usage
        if input_queue.qsize() > 0:
            input_reading = input_queue.get()

            if input_reading == "S":
                serial_handler.write("S".encode())
                # reset the nested list system when we start a new run
                data_recorder = [["t", "ml", "mr", "sl", "sr", "p", "i", "d"]]
            elif input_reading == "E":
                serial_handler.write("E".encode())
                # convert to dataframe and write to a csv when we stop the program!
                df = pd.DataFrame(data_recorder[1:], columns=data_recorder[0])
                df.to_csv("robot_run", sep=',', encoding='utf-8')
                # print it out to see if that worked
                print(df)
            elif input_reading[0] == "P":
                # change the proportional constant
                output = "P" + input_reading[1:]
                serial_handler.write(output.encode())
            elif input_reading[0] == "I":
                # change the integral constant
                output = "I" + input_reading[1:]
                serial_handler.write(output.encode())
            elif input_reading[0] == "D":
                # change the derivative constant
                output = "D" + input_reading[1:]
                serial_handler.write(output.encode())
            elif input_reading[0] == "V":
                # change the speed of the robot
                output = "V" + input_reading[1:]
                serial_handler.write(output.encode())


if __name__ == '__main__':
    main()
