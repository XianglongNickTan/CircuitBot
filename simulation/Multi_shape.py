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
import numpy as np
import random
from shapely.geometry import Point, LineString, Polygon, MultiLineString

# from bayes_opt import SequentialDomainReductionTransformer

# Bound region of parameter space
# For now, we have 6 parameters. The range of x is (-0.21, 0.1), the range of y is (-0.55, -0.4)


xybounds = {'s1': (0, 5), 'x1': (-12, -9), 'y1': (-2, 2), 's2': (0, 5), 'x2': (-8, -4), 'y2': (-2, 2),
            's3': (0, 5), 'x3': (-2, 2), 'y3': (-2, 2), 's4': (0, 5), 'x4': (4, 8), 'y4': (-2, 2),
            's5': (0, 5), 'x5': (9, 12), 'y5': (-2, 2)}

# xybounds = {'s1': (0, 5), 'x1': (-12, 12), 'y1': (-5, 5), 's2': (0, 5), 'x2': (-12, 12), 'y2': (-5, 5),
#             's3': (0, 5), 'x3': (-12, 12), 'y3': (-5, 5), 's4': (0, 5), 'x4': (-12, 12), 'y4': (-5, 5),
#             's5': (0, 5), 'x5': (-12, 12), 'y5': (-5, 5)}




def draw_shape(indicator, xy_center_pos, size=5):
    if indicator == 0:  # hor_line
        p = LineString([(xy_center_pos[0] - size, xy_center_pos[1]), (xy_center_pos[0] + size, xy_center_pos[1])])

    elif indicator == 1:  # ver_line
        p = LineString([(xy_center_pos[0], xy_center_pos[1] - size), (xy_center_pos[0], xy_center_pos[1] + size)])
        # p = LineString([(xy_center_pos[0] - size, xy_center_pos[1]), (xy_center_pos[0] + size, xy_center_pos[1])])

    elif indicator == 2:  # cross
        p = MultiLineString(
            [((xy_center_pos[0] + size, xy_center_pos[1] + size), (xy_center_pos[0] - size, xy_center_pos[1] - size)),
             ((xy_center_pos[0] - size, xy_center_pos[1] + size), (xy_center_pos[0] + size, xy_center_pos[1] - size))])

    elif indicator == 3:  # circle
        p = Point(xy_center_pos[0], xy_center_pos[1]).buffer(size)

    elif indicator == 4:  # triangle
        size = 9*np.sqrt(3)/3
        point_1 = [xy_center_pos[0] - size * 2 * np.sqrt(3) / 3, xy_center_pos[1]]
        point_2 = [xy_center_pos[0] + size * np.sqrt(3) / 3, xy_center_pos[1] - size]
        point_3 = [xy_center_pos[0] + size * np.sqrt(3) / 3, xy_center_pos[1] + size]
        p = Polygon([point_1, point_2, point_3])

    elif indicator == 5:  # diamond
        size = 6
        point_1 = (xy_center_pos[0] + size, xy_center_pos[1])
        point_2 = (xy_center_pos[0], xy_center_pos[1] - size)
        point_3 = (xy_center_pos[0] - size, xy_center_pos[1])
        point_4 = (xy_center_pos[0], xy_center_pos[1] + size)
        p = Polygon([point_1, point_2, point_3, point_4])

    return p


def circuitBot():

    optimizer = BayesianOptimization(
        f=None,
        pbounds=xybounds,
        verbose=2,  # choices: 0, 1, 2. verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
        random_state=108,
        # random_state=1,
        # bounds_transformer = SequentialDomainReductionTransformer()
    )
    # For kind = "ucb", small kappa (1) prefer exploitation, big kappa (10) prefer exploration
    # For kind = "ei", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration
    # kind = "poi", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration

    logger = JSONLogger(path="./Logs.json")
    optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

    utility = UtilityFunction(kind="ucb", kappa=2, xi=0.0)

    # To output paramters and voltage as a csv file

    bar1_x = -13
    bar2_x = 13
    shape_num = 5
    bar1 = draw_shape(1, [bar1_x, 0], size=13)
    bar2 = draw_shape(1, [bar2_x, 0], size=13)
    # shape_count_list = []
    count = 0
    iteration = 0
    count_part = 0
    parameter = []
    shape_count_list = []

    for _ in range(100):
    # while(1):
        iteration += 1
        state_list = []
        intersection_vec = []
        left_connect = False
        right_connect = False
        all_connect = False
        shape = []
        reward = 0
        next_to_probe = optimizer.suggest(utility)  # A dict to tell you the next parameters to probe

        for item in next_to_probe:
            next_to_probe[item] = round(next_to_probe[item])
            state_list.append(next_to_probe[item])

        shape_list = state_list[0:shape_num]
        x_list = state_list[shape_num:2 * shape_num]
        y_list = state_list[2 * shape_num:]

        shape.append(bar1)
        for i in range(shape_num):
            shape.append(draw_shape(shape_list[i], [x_list[i], y_list[i]]))
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

        connect_list = []

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
            state =[]
            count += 1
            reward = 100
            shape_count_list.append(shape_list)
            # state.append(next_to_probe)
            state = ""
            for i in range(5):
                state += str(state_list[i])
                state += ','
                state += ' '

            for j in range(5):
                state += str(state_list[j+5])
                state += ','
                state += ' '
                state += str(state_list[j+10])
                state += ','
                state += ' '

            parameter.append(state)




        # if count == 20:
        #     break

        # else:
        #     reward = 0
        #
        # try:
        #     optimizer.register(
        #         params=next_to_probe,
        #         target=float(reward)
        #     )
        # except KeyError:
        #     next_to_probe['x1'] = next_to_probe['x1'] + random.randint(1, 1000)*0.001
        #     optimizer.register(
        #         params=next_to_probe,
        #         target=float(reward)
        #     )
    print(count)
    print(iteration)
    print(shape_count_list)
    print(parameter)


    # point_file = open("next.txt", "w")
    # for item in parameter:
    #     point_str = str()
    #
    #     for key in item:
    #         point_str += str(item[key])
    #         point_str += " "
    #     # print(point_str)
    #
    # # point_file.write(point_str)
    #
    # point_file.close()


if __name__ == "__main__":
    circuitBot()
    print("Done")
    # print(a)
