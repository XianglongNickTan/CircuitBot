# -*- coding: utf-8 -*-
#==========================================
# Title:  syntheticFunctions.py
# Author: Binxin Ru and Ahsan Alvi
# Date:   20 August 2019
# Link:   https://arxiv.org/abs/1906.08878
#==========================================

import numpy as np
from utils.connection import check_connection


def read_voltage(x):
    state_list = []
    point_str = str()

    for item in x:
        # x[item] = round(x[item])
        state_list.append(item)
        point_str += str(item)
        point_str += " "

    if check_connection(state_list, multi_shape=True):
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
