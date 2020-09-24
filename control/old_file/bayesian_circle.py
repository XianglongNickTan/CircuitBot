# -*- coding: utf-8 -*-
import os
import time
import pandas as pd
import random
from bayes_opt import BayesianOptimization
from bayes_opt import UtilityFunction
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
from bayes_opt.util import load_logs
from shapely.geometry import Point, LineString, Polygon, MultiLineString
import numpy as np

# from bayes_opt import SequentialDomainReductionTransformer

# Bound region of parameter space
# For now, we have 6 parameters. The range of x is (-0.21, 0.1), the range of y is (-0.55, -0.4)

xybounds = {'x1': (-12, 12), 'y1': (-5, 5), 'x2': (-12, 12), 'y2': (-5, 5),
            'x3': (-12, 12), 'y3': (-5, 5), 'x4': (-12, 12), 'y4': (-5, 5),
            'x5': (-12, 12), 'y5': (-5, 5)}

# Add the offset to convert to real world coordinates (cm)
x_offset = -4
y_offset = -53
load_resistance = 45
supply_voltage = 45

bar1_x = -13
bar2_x = 13
shape_num = 5


def draw_shape(indicator, xy_center_pos, size=5):
    if indicator == 0:  # hor_line
        p = LineString([(xy_center_pos[0] - size, xy_center_pos[1]), (xy_center_pos[0] + size, xy_center_pos[1])])

    if indicator == 1:  # ver_line
        p = LineString([(xy_center_pos[0], xy_center_pos[1] - size), (xy_center_pos[0], xy_center_pos[1] + size)])

    if indicator == 2:  # cross
        p = MultiLineString(
            [((xy_center_pos[0] - size, xy_center_pos[1]), (xy_center_pos[0] + size, xy_center_pos[1])),
             ((xy_center_pos[0], xy_center_pos[1] - size), (xy_center_pos[0], xy_center_pos[1] + size))])

    if indicator == 3:  # circle
        p = Point(xy_center_pos[0], xy_center_pos[1]).buffer(size)

    if indicator == 4:  # triangle
        point_1 = (xy_center_pos[0], xy_center_pos[1] + size * 2 * np.sqrt(3) / 3)
        point_2 = (xy_center_pos[0] - size, xy_center_pos[1] - size * np.sqrt(3) / 3)
        point_3 = (xy_center_pos[0] + size, xy_center_pos[1] - size * np.sqrt(3) / 3)
        p = Polygon([point_1, point_2, point_3])

    if indicator == 5:  # square
        point_1 = (xy_center_pos[0] + size, xy_center_pos[1] - size)
        point_2 = (xy_center_pos[0] - size, xy_center_pos[1] - size)
        point_3 = (xy_center_pos[0] - size, xy_center_pos[1] + size)
        point_4 = (xy_center_pos[0] + size, xy_center_pos[1] + size)
        p = Polygon([point_1, point_2, point_3, point_4])

    return p


def check_connection(state_list, shape_num=shape_num, bar1_x=bar1_x, bar2_x=bar2_x, multi_shape=False):
    intersection_vec = []
    left_connect = False
    right_connect = False
    all_connect = False
    shape = []

    bar1 = draw_shape(1, [bar1_x, 0], size=13)
    bar2 = draw_shape(1, [bar2_x, 0], size=13)

    if multi_shape:
        shape_list = state_list[0:shape_num]
        x_list = state_list[shape_num:2 * shape_num]
        y_list = state_list[2 * shape_num:]
        shape.append(bar1)
        for i in range(shape_num):
            shape.append(draw_shape(shape_list[i], [x_list[i], y_list[i]]))
        shape.append(bar2)

    else:
        x_list = state_list[0:shape_num]
        y_list = state_list[shape_num:]
        shape.append(bar1)
        for i in range(shape_num):
            shape.append(draw_shape(3, [x_list[i], y_list[i]]))
        shape.append(bar2)

    for j in range(shape_num + 2):
        intersection = []
        for k in range(shape_num + 2):
            if k != j:
                flag = shape[j].intersects(shape[k])
                if flag:
                    intersection.append(k)
        intersection_vec.append(intersection)

    if intersection_vec[0]:
        left_connect = True

    if intersection_vec[shape_num + 1]:
        right_connect = True

    def del_rep_shape(connected_list, target_list):
        for item in target_list:
            if item in connected_list:
                target_list.remove(item)

    if left_connect & right_connect:
        for item in intersection_vec[0]:
            connect_list_0 = [0, item]
            touch = intersection_vec[item].copy()
            del_rep_shape(connect_list_0, touch)
            if touch:
                if (shape_num + 1) in touch:
                    all_connect = True
                    connect_list = connect_list_0
                for item in touch:
                    connect_list_1 = connect_list_0.copy()
                    connect_list_1.append(item)
                    touch_1 = intersection_vec[item].copy()
                    del_rep_shape(connect_list_1, touch_1)
                    if touch_1:
                        if (shape_num + 1) in touch_1:
                            all_connect = True
                            connect_list = connect_list_1
                        for item in touch_1:
                            connect_list_2 = connect_list_1.copy()
                            connect_list_2.append(item)
                            touch_2 = intersection_vec[item].copy()
                            del_rep_shape(connect_list_2, touch_2)
                            if touch_2:
                                if (shape_num + 1) in touch_2:
                                    all_connect = True
                                    connect_list = connect_list_2
                                for item in touch_2:
                                    connect_list_3 = connect_list_2.copy()
                                    connect_list_3.append(item)
                                    touch_3 = intersection_vec[item].copy()
                                    del_rep_shape(connect_list_3, touch_3)
                                    if touch_3:
                                        if (shape_num + 1) in touch_3:
                                            all_connect = True
                                            connect_list = connect_list_3
                                        for item in touch_3:
                                            connect_list_4 = connect_list_3.copy()
                                            connect_list_4.append(item)
                                            touch_4 = intersection_vec[item].copy()
                                            del_rep_shape(connect_list_4, touch_4)
                                            if touch_4:
                                                if (shape_num + 1) in touch_4:
                                                    all_connect = True
                                                    connect_list = connect_list_4
                                                for item in touch_4:
                                                    connect_list_5 = connect_list_4.copy()
                                                    connect_list_5.append(item)
                                                    touch_5 = intersection_vec[item].copy()
                                                    del_rep_shape(connect_list_5, touch_5)
                                                    if touch_5:
                                                        if (shape_num + 1) in touch_5:
                                                            all_connect = True
                                                            connect_list = connect_list_5
                                                        for item in touch_5:
                                                            connect_list_6 = connect_list_5.copy()
                                                            connect_list_6.append(item)
                                                            touch_6 = intersection_vec[item].copy()
                                                            del_rep_shape(connect_list_6, touch_6)
                                                            if touch_6:
                                                                if (shape_num + 1) in touch_6:
                                                                    all_connect = True
                                                                    connect_list = connect_list_6

    # print(intersection_vec)
    # print(connect_list)
    if all_connect:
        return True

    else:
        return False


def circuitBot():
    optimizer = BayesianOptimization(
        f=None,
        pbounds=xybounds,
        verbose=2,  # choices: 0, 1, 2. verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
        random_state=108,
        # bounds_transformer = SequentialDomainReductionTransformer()
    )
    # For kind = "ucb", small kappa (1) prefer exploitation, big kappa (10) prefer exploration
    # For kind = "ei", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration
    # kind = "poi", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration

    logger = JSONLogger(path="./Logs.json")
    optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

    utility = UtilityFunction(kind="ucb", kappa=5, xi=0.0)

    # To output paramters and voltage as a csv file
    name = ['x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'x4', 'y4', 'x5', 'y5', 'voltage', 'resistance']
    csv_list = []

    # pretrainning part

    probe1 = {'x1': -2, 'y1': -4, 'x2': 5, 'y2': -3, 'x3': -12, 'y3': -2, 'x4': -5, 'y4': -1, 'x5': -8, 'y5': 0}
    probe2 = {'x1': 7, 'y1': 4, 'x2': 11, 'y2': -4, 'x3': -4, 'y3': -5, 'x4': 5, 'y4': -3, 'x5': 9, 'y5': 4}
    probe3 = {'x1': 12, 'y1': -1, 'x2': 6, 'y2': 4, 'x3': -5, 'y3': -2, 'x4': 7, 'y4': -2, 'x5': -10, 'y5': -4}
    probe4 = {'x1': -12, 'y1': -4, 'x2': 4, 'y2': 1, 'x3': -7, 'y3': -4, 'x4': -6, 'y4': 1, 'x5': 0, 'y5': 2}
    probe5 = {'x1': -10, 'y1': 0, 'x2': -2, 'y2': 2, 'x3': 5, 'y3': 0, 'x4': -2, 'y4': 4, 'x5': -11, 'y5': 1}
    probe6 = {'x1': -2, 'y1': 2, 'x2': 4, 'y2': -1, 'x3': -7, 'y3': 1, 'x4': 9, 'y4': -4, 'x5': -11, 'y5': -3}
    probe7 = {'x1': -10, 'y1': -2, 'x2': -2, 'y2': 2, 'x3': 11, 'y3': 3, 'x4': 1, 'y4': -5, 'x5': 5, 'y5': 3}
    probe8 = {'x1': 10, 'y1': -3, 'x2': -9, 'y2': 4, 'x3': -9, 'y3': -2, 'x4': 7, 'y4': 3, 'x5': -2, 'y5': 2}
    probe9 = {'x1': -10, 'y1': 2, 'x2': 11, 'y2': -3, 'x3': 2, 'y3': 1, 'x4': -7, 'y4': 5, 'x5': -6, 'y5': 3}
    probe10 = {'x1': -6, 'y1': -5, 'x2': 0, 'y2': -4, 'x3': 3, 'y3': 0, 'x4': 8, 'y4': 1, 'x5': -8, 'y5': 1}
    # probe11 = {'x1': 8, 'y1': 0, 'x2': 4, 'y2': 0, 'x3': 0, 'y3': 0, 'x4': -4, 'y4': 0, 'x5': -8, 'y5': 0}
    # probe12 = {'x1': 12, 'y1': 0, 'x2': 6, 'y2': 0, 'x3': 0, 'y3': 0, 'x4': -6, 'y4': 0, 'x5': -12, 'y5': 0}

    probes = [probe1, probe2, probe3, probe4, probe5, probe6, probe7, probe8, probe9, probe10]
    # probes = [probe1, probe2, probe3, probe4, probe5, probe6, probe7, probe8, probe9, probe10, probe11, probe12]
    results = [0, 0, 0, 0, 0, 25.7, 21.1, 23.9, 18.3, 25.8]
    # results = [0, 0, 0, 0, 0, 17.1, 14.1, 15.9, 12.2, 17.2, 17.3, 16.0]
    #
    # input_path = "./data.csv"
    # initData = pd.read_csv("./his_data.csv")
    # for i in initData.index:
    #     temp_probe = {}
    #     temp_probe['x1'] = initData['x1'][i]
    #     temp_probe['y1'] = initData['y1'][i]
    #     temp_probe['x2'] = initData['x2'][i]
    #     temp_probe['y2'] = initData['y2'][i]
    #     temp_probe['x3'] = initData['x3'][i]
    #     temp_probe['y3'] = initData['y3'][i]
    #     temp_probe['x4'] = initData['x4'][i]
    #     temp_probe['y4'] = initData['y4'][i]
    #     temp_probe['x5'] = initData['x5'][i]
    #     temp_probe['y5'] = initData['y5'][i]
    #     probes.append(temp_probe)
    #     results.append(initData['voltage'][i])
    #


    for j in range(10):
        optimizer.register(params=probes[j], target=results[j])

    # train part
    for i in range(50):
        state_list = []

        next_to_probe = optimizer.suggest(utility)  # A dict to tell you the next parameters to probe
        # 将连续值变为离散值
        for item in next_to_probe:
            next_to_probe[item] = round(next_to_probe[item])
            state_list.append(next_to_probe[item])

        point_file = open("next.txt", "w")
        point_str = str()
        for key in next_to_probe:
            point_str += str(next_to_probe[key])
            point_str += " "
        print(point_str)
        print("No of trial :", i)

        point_file.write(point_str)
        point_file.close()

        if check_connection(state_list):
            print('Connected!')

        else:
            print('No connection!')

        print("Now, change to the other terminal to let robot arm draw.")
        # print("Press q when it finished.")

        ###################################################
        # Now we should wait the robot arm draw circle
        # Pass q when the robot arm finished
        ###################################################

        # keyboard.wait("q")
        while True:
            if input("Press q when it finished:") == "q":
                break

        voltage_file = open("voltage.txt", "r")
        voltage = voltage_file.read()
        voltage_file.close()

        voltage = float(voltage)

        # calculate resistance
        if voltage == 0.0:
            r = 1000.0
        else:
            r = load_resistance * supply_voltage / voltage - load_resistance
            r = round(r)

        try:
            optimizer.register(
                params=next_to_probe,
                target=voltage
            )
        except KeyError:
            next_to_probe['x1'] = next_to_probe['x1'] + random.randint(1, 1000) * 0.00001
            optimizer.register(
                params=next_to_probe,
                target=voltage
            )

        # To record these parameter and voltage
        tmp = []
        for key in next_to_probe:
            tmp.append(next_to_probe[key])
        tmp.append(voltage)
        tmp.append(r)
        csv_list.append(tmp)

    csv = pd.DataFrame(columns=name, data=csv_list)
    output_file = "data.csv"
    csv.to_csv(output_file)


if __name__ == "__main__":
    circuitBot()