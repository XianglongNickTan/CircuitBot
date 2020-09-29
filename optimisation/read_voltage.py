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

    hor_line = state_list.count(0)
    ver_line = state_list.count(1)
    cross = state_list.count(2)
    circle = state_list.count(3)
    triangle = state_list.count(4)
    diamond = state_list.count(5)

    if check_connection(state_list):
        # voltage = ver_line *100
        # if ver_line == 0:
        #     voltage = 100
        #     result = -voltage
        #
        # else:
        #     result = 100
        voltage = 100
        result = -voltage

        print('connected')

    else:
        result = 0.0
        print('no connection')

    return result

def read_voltage_line_circle(x):
    point_str = str()

    for item in x:
        point_str += str(item)
        point_str += " "

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

    return result

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
