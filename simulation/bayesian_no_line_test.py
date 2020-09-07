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
# xybounds = {'x1': (-0.15, 0.09), 'y1': (-0.6, -0.46), 'x2': (-0.15, 0.09), 'y2': (-0.6, -0.46), 'x3': (-0.15, 0.09),
#             'y3': (-0.6, -0.46)}
xybounds = {'s1': (0, 5), 'x1': (-15, 9), 'y1': (-60, -46), 's2': (0, 5), 'x2': (-15, 9), 'y2': (-60, -46),
            's3': (0, 5), 'x3': (-15, 9), 'y3': (-60, -46), 's4': (0, 5), 'x4': (-15, 9), 'y4': (-60, -46),
            's5': (0, 5), 'x5': (-15, 9), 'y5': (-60, -46), 's6': (0, 5), 'x6': (-15, 9), 'y6': (-60, -46)}

# Times of experiment
times_of_experiments = 10


def draw_shape(indicator, xy_center_pos, size=6):
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


def circuitBot():

    optimizer = BayesianOptimization(
        f=None,
        pbounds=xybounds,
        verbose=2,  # choices: 0, 1, 2. verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
        random_state=int(random.random() * 100),
        # random_state=1,
        # bounds_transformer = SequentialDomainReductionTransformer()
    )
    # For kind = "ucb", small kappa (1) prefer exploitation, big kappa (10) prefer exploration
    # For kind = "ei", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration
    # kind = "poi", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration

    logger = JSONLogger(path="./Logs.json")
    optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

    utility = UtilityFunction(kind="ucb", kappa=1, xi=0.0)

    # To output paramters and voltage as a csv file
    name = ['x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'voltage']
    x_list = []
    radius = 6
    # bar1_x = 15
    # bar2_x = -20
    bar1_x = -18
    bar2_x = 12
    # bar1_x = -5
    # bar2_x = 3
    shape_num = 6
    bar1 = draw_shape(1, [bar1_x, -53], size=13)
    bar2 = draw_shape(1, [bar2_x, -53], size=13)

    a = 0

    for _ in range(1):
        state_list = []
        x_list = []
        y_list = []
        shape_list = []
        intersection_vec = []
        left_connect = False
        right_connect = False
        all_connect = False
        shape = []
        reward = 0
        next_to_probe = optimizer.suggest(utility)  # A dict to tell you the next parameters to probe
        # print(next_to_probe)
        for item in next_to_probe:
            next_to_probe[item] = round(next_to_probe[item])
            state_list.append(next_to_probe[item])
            # print(xy_list)
        shape_list = state_list[0:6]
        x_list = state_list[6:12]
        y_list = state_list[12:]
        x_list_sorted = sorted(x_list)

        # # x_list_sorted = sorted(x_list, reverse=True)
        shape.append(bar1)
        for i in range(len(shape_list)):
            shape.append(draw_shape(shape_list[i], [x_list[i], y_list[i]]))
        shape.append(bar2)

        # for j in range(len(shape)):
        #     intersection = [0 for _ in range(len(shape))]
        #     for k in range(len(shape)):
        #         if k != j:
        #             flag = shape[j].intersects(shape[k])
        #             if flag:
        #                 intersection[k] = 1
        #     intersection_vec.append(intersection)

        for j in range(shape_num + 2):
            intersection = []
            for k in range(shape_num + 2):
                if k != j:
                    flag = shape[j].intersects(shape[k])
                    if flag:
                        intersection.append(k)
            intersection_vec.append(intersection)
        print(intersection_vec)

        if intersection_vec[0]:
            left_connect = True

        if intersection_vec[shape_num + 1]:
            right_connect = True

        def del_rep_shape(connect_list, target_list):
            for item in target_list:
                if item in connect_list:
                    target_list.remove(item)

        connect_list = []

        if left_connect & right_connect:
            a += 1
            for item in intersection_vec[0]:
                connect_list_0 = [0, item]
                touch = intersection_vec[item].copy()
                del_rep_shape(connect_list_0, touch)
                if touch:
                    if (shape_num + 1) in touch:
                        all_connect = True
                        connect_list = connect_list_0
                        print("successful")
                    for item in touch:
                        # print(touch)
                        connect_list_1 = connect_list_0.copy()
                        connect_list_1.append(item)
                        touch_1 = intersection_vec[item].copy()
                        del_rep_shape(connect_list_1, touch_1)
                        if touch_1:
                            if (shape_num + 1) in touch_1:
                                all_connect = True
                                connect_list = connect_list_1
                                print("successful")

                            for item in touch_1:
                                connect_list_2 = connect_list_1.copy()
                                connect_list_2.append(item)
                                touch_2 = intersection_vec[item].copy()
                                del_rep_shape(connect_list_2, touch_2)
                                if touch_2:
                                    if (shape_num + 1) in touch_2:
                                        all_connect = True
                                        connect_list = connect_list_2
                                        print("successful")

                                    for item in touch_2:
                                        connect_list_3 = connect_list_2.copy()
                                        connect_list_3.append(item)
                                        touch_3 = intersection_vec[item].copy()
                                        del_rep_shape(connect_list_3, touch_3)
                                        if touch_3:
                                            if (shape_num + 1) in touch_3:
                                                all_connect = True
                                                connect_list = connect_list_3
                                                print("successful")

                                            for item in touch_3:
                                                connect_list_4 = connect_list_3.copy()
                                                connect_list_4.append(item)
                                                touch_4 = intersection_vec[item].copy()
                                                del_rep_shape(connect_list_4, touch_4)
                                                if touch_4:
                                                    # print("4")
                                                    if (shape_num + 1) in touch_4:
                                                        all_connect = True
                                                        connect_list = connect_list_4
                                                        print("successful")

                                                    for item in touch_4:
                                                        connect_list_5 = connect_list_4.copy()
                                                        connect_list_5.append(item)
                                                        touch_5 = intersection_vec[item].copy()
                                                        del_rep_shape(connect_list_5, touch_5)
                                                        if touch_5:
                                                            if (shape_num + 1) in touch_5:
                                                                all_connect = True
                                                                connect_list = connect_list_5
                                                                print("successful")

                                                            for item in touch_5:
                                                                connect_list_6 = connect_list_5.copy()
                                                                connect_list_6.append(item)
                                                                touch_6 = intersection_vec[item].copy()
                                                                del_rep_shape(connect_list_6, touch_6)
                                                                if touch_6:
                                                                    if (shape_num + 1) in touch_6:
                                                                        all_connect = True
                                                                        connect_list = connect_list_6
                                                                        print("successful")

        print(intersection_vec)
        print(connect_list)
        # if all_connect:
            # print("successful")


        #
        # print(all_connect)
        # print(connect_list)


        #
        #
        # for i in range(len(shape_list)):
        #     flag_0 = bar1.intersects(shape[i])
        #     if flag_0:
        #         reward += 10
        #         for j in range(len(shape_list)):
        #             if j != i:
        #                 flag_1 = shape[i].intersects(shape[j])
        #                 if flag_1:





        # dis_1 = np.float(bar1_x - x_list_sorted[0])
        # dis_2 = np.float(x_list_sorted[2] - bar2_x)
        # dis_3 = np.square(xy_list[0] - xy_list[1]) + np.square(xy_list[3] - xy_list[4])
        # dis_4 = np.square(xy_list[0] - xy_list[2]) + np.square(xy_list[3] - xy_list[5])
        # dis_5 = np.square(xy_list[1] - xy_list[2]) + np.square(xy_list[4] - xy_list[5])
        #
        # if (dis_1 <= radius) & (dis_2 <= radius) & (dis_3 <= radius) & (dis_4 <= radius) & (dis_5 <= radius):
        #     reward = 10
        #
        # else:
        #     reward = 0
        #
        # # optimizer.register(
        # #     params=next_to_probe,
        # #     target=reward
        # # )
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

    print(a)

if __name__ == "__main__":
    circuitBot()
    print("Done")
    # print(a)
