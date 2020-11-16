# -*- coding: utf-8 -*-
#==========================================
# Title:  syntheticFunctions.py
# Author: Binxin Ru and Ahsan Alvi
# Date:   20 August 2019
# Link:   https://arxiv.org/abs/1906.08878
#==========================================

import numpy as np
from utils.connection import check_connection
from collections import Counter
import random


def read_voltage_simulation(x):
    state_list = []
    point_str = str()

    for item in x:
        # x[item] = round(x[item])
        state_list.append(item)

    # seed = int(np.round((np.random.random() + 1) * 100))

    # np.random.seed(108)
    # random.seed(108)

    # print(state_list)
    line = state_list.count(0)
    circle = state_list.count(1)

    #
    if state_list[2] == 1:
        resistance = 360
        voltage = 3
        print('..................')
        print('shit')
        print('..................')

    elif (state_list[5] < 6) & (state_list[1] == 1):
        voltage = 9
        print('..................')
        print('shit')
        print('..................')

    elif (state_list[7] > -6) & (state_list[3] == 1):
        voltage = 9
        print('..................')
        print('shit')
        print('..................')
    else:
        resistance = 80 - circle * 6 + line * 3
        voltage = (11 + circle * 0.8) - 0.8 + random.random()

    # voltage = (11 + circle * 0.8) - 0.8 + random.random()


    # resistance = (80 - circle * 6 + random.random()) * -1
    # resistance = 80 - circle * 6 + random.random() - 60

    # resistance = -1 * pow(10, circle)

    return voltage

def read_voltage_line_circle(x):
    point_str = str()

    for item in x:
        point_str += str(item)
        point_str += " "

    # print("------------------------------")
    # print(point_str)
    # print("------------------------------")
    point_file = open("next.txt", "w")

    point_file.write(point_str)
    point_file.close()


    while True:
        if input("Press q when it finished:") == "q":
            break

    voltage_file = open("voltage.txt", "r")
    voltage = voltage_file.read()
    voltage_file.close()
    voltage = (float(voltage))
    # resistance = (30 - voltage) / voltage * 45

    return voltage

def read_voltage_obstacle(x):
    state_list = []
    point_str = str()

    for item in x:
        # x[item] = round(x[item])
        state_list.append(item)
        point_str += str(item)
        point_str += " "

    if check_connection(state_list):
        print("------------------------------")
        print(point_str)
        print("------------------------------")
        point_file = open("next.txt", "w")

        point_file.write(point_str)
        point_file.close()


        while True:
            if input("Press q when it finished:") == "q":
                break

        voltage_file = open("voltage.txt", "r")
        voltage = voltage_file.read()
        voltage_file.close()
        voltage = float(voltage)
        result = -voltage

    else:
        result = 0.0

    return result
