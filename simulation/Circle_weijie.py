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
# xybounds = {'s1': (0, 5), 'x1': (-15, 9), 'y1': (-60, -46), 's2': (0, 5), 'x2': (-15, 9), 'y2': (-60, -46),
#             's3': (0, 5), 'x3': (-15, 9), 'y3': (-60, -46), 's4': (0, 5), 'x4': (-15, 9), 'y4': (-60, -46),
#             's5': (0, 5), 'x5': (-15, 9), 'y5': (-60, -46), 's6': (0, 5), 'x6': (-15, 9), 'y6': (-60, -46)}

# xybounds = {'s1': (0, 5), 'x1': (-12, 12), 'y1': (-7, 7), 's2': (0, 5), 'x2': (-12, 12), 'y2': (-7, 7),
#             's3': (0, 5), 'x3': (-12, 12), 'y3': (-7, 7), 's4': (0, 5), 'x4': (-12, 12), 'y4': (-7, 7),
#             's5': (0, 5), 'x5': (-12, 12), 'y5': (-7, 7)}
#
xybounds = {'x1': (-12, 12), 'y1': (-5, 5), 'x2': (-12, 12), 'y2': (-5, 5),
            'x3': (-12, 12), 'y3': (-5, 5), 'x4': (-12, 12), 'y4': (-5, 5),
            'x5': (-12, 12), 'y5': (-5, 5)}

# xybounds = {'x1': (-12, 12), 'y1': (-5, 5), 'x2': (-12, 12), 'y2': (-5, 5),
#             'x3': (-12, 12), 'y3': (-5, 5), 'x4': (-12, 12), 'y4': (-5, 5),
#             'x5': (-12, 12), 'y5': (-5, 5), 'x6': (-12, 12), 'y6': (-5, 5)}

# Times of experiment
times_of_experiments = 10


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


def circuitBot():

    optimizer = BayesianOptimization(
        f=None,
        pbounds=xybounds,
        verbose=2,  # choices: 0, 1, 2. verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
        # random_state=int(random.random() * 100),
        random_state=108,
        # bounds_transformer = SequentialDomainReductionTransformer()
    )
    # For kind = "ucb", small kappa (1) prefer exploitation, big kappa (10) prefer exploration
    # For kind = "ei", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration
    # kind = "poi", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration

    logger = JSONLogger(path="./Logs.json")
    optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

    utility = UtilityFunction(kind="ucb", kappa=2, xi=0.0)

    # To output paramters and voltage as a csv file
    name = ['x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'voltage']
    x_list = []
    # bar1_x = 15
    # bar2_x = -20
    bar1_x = -13
    bar2_x = 13
    # bar1_x = -5
    # bar2_x = 3
    shape_num = 5
    bar1 = draw_shape(1, [bar1_x, 0], size=13)
    bar2 = draw_shape(1, [bar2_x, 0], size=13)

    count = 0
    count_part = 0
    #
    # probe1 = {'x1': -2, 'y1': -4, 'x2': 5, 'y2': -3, 'x3': -12, 'y3': -2, 'x4': -5, 'y4': -1, 'x5': -8, 'y5': 0}
    # probe2 = {'x1': 7, 'y1': 4, 'x2': 11, 'y2': -4, 'x3': -4, 'y3': -5, 'x4': 5, 'y4': -3, 'x5': 9, 'y5': 4}
    # probe3 = {'x1': 12, 'y1': -1, 'x2': 6, 'y2': 4, 'x3': -5, 'y3': -2, 'x4': 7, 'y4': -2, 'x5': -10, 'y5': -4}
    # probe4 = {'x1': -12, 'y1': -4, 'x2': 4, 'y2': 1, 'x3': -7, 'y3': -4, 'x4': -6, 'y4': 1, 'x5': 0, 'y5': 2}
    # probe5 = {'x1': -10, 'y1': 0, 'x2': -2, 'y2': 2, 'x3': 5, 'y3': 0, 'x4': -2, 'y4': 4, 'x5': -11, 'y5': 1}
    # probe6 = {'x1': -2, 'y1': 2, 'x2': 4, 'y2': -1, 'x3': -7, 'y3': 1, 'x4': 9, 'y4': -4, 'x5': -11, 'y5': -3}
    # probe7 = {'x1': -10, 'y1': -2, 'x2': -2, 'y2': 2, 'x3': 11, 'y3': 3, 'x4': 1, 'y4': -5, 'x5': 5, 'y5': 3}
    # probe8 = {'x1': 10, 'y1': -3, 'x2': -9, 'y2': 4, 'x3': -9, 'y3': -2, 'x4': 7, 'y4': 3, 'x5': -2, 'y5': 2}
    # probe9 = {'x1': -10, 'y1': 2, 'x2': 11, 'y2': -3, 'x3': 2, 'y3': 1, 'x4': -7, 'y4': 5, 'x5': -6, 'y5': 3}
    # probe10 = {'x1': -6, 'y1': -5, 'x2': 0, 'y2': -4, 'x3': 3, 'y3': 0, 'x4': 8, 'y4': 1, 'x5': -8, 'y5': 1}
    # probes = [probe1, probe2, probe3, probe4, probe5, probe6, probe7, probe8, probe9, probe10]
    # results = [0, 0, 0, 0, 0, 100, 100, 100, 100, 100]
    #
    #
    # for j in range(10):
    #     optimizer.register(params=probes[j], target=results[j])

    # Provide some initial points for the algorithm to learn first
    probe1 = {'x1': 0.04, 'y1': -0.53, 'x2': 0.00, 'y2': -0.53, 'x3': -0.04, 'y3': -0.53,
              'x4': -0.08, 'y4': -0.53, 'x5': -0.12, 'y5': -0.53}
    probe2 = {'x1': 0.08, 'y1': -0.53, 'x2': 0.02, 'y2': -0.53, 'x3': -0.04, 'y3': -0.53,
              'x4': -0.10, 'y4': -0.53, 'x5': -0.16, 'y5': -0.53}
    probe3 = {'x1': -0.06, 'y1': -0.51, 'x2': 0.00, 'y2': -0.54, 'x3': -0.11, 'y3': -0.52,
              'x4': 0.05, 'y4': -0.57, 'x5': -0.15, 'y5': -0.56}
    probe4 = {'x1': -0.14, 'y1': -0.55, 'x2': -0.06, 'y2': -0.51, 'x3': 0.07, 'y3': -0.50,
              'x4': -0.03, 'y4': -0.58, 'x5': 0.01, 'y5': -0.50}
    probe5 = {'x1': 0.06, 'y1': -0.56, 'x2': -0.13, 'y2': -0.49, 'x3': -0.13, 'y3': -0.55,
              'x4': 0.03, 'y4': -0.5, 'x5': -0.06, 'y5': -0.51}
    probe6 = {'x1': -0.14, 'y1': -0.51, 'x2': 0.07, 'y2': -0.56, 'x3': -0.02, 'y3': -0.52,
              'x4': -0.11, 'y4': -0.48, 'x5': -0.1, 'y5': -0.50}
    probe7 = {'x1': -0.10, 'y1': -0.58, 'x2': -0.04, 'y2': -0.57, 'x3': -0.01, 'y3': -0.53,
              'x4': 0.04, 'y4': -0.52, 'x5': -0.12, 'y5': -0.52}
    probe8 = {'x1': -0.06, 'y1': -0.57, 'x2': 0.01, 'y2': -0.56, 'x3': -0.16, 'y3': -0.55,
              'x4': -0.09, 'y4': -0.54, 'x5': -0.12, 'y5': -0.53}
    probe9 = {'x1': 0.03, 'y1': -0.49, 'x2': 0.07, 'y2': -0.57, 'x3': -0.08, 'y3': -0.58,
              'x4': 0.01, 'y4': -0.56, 'x5': 0.05, 'y5': -0.50}
    probe10 = {'x1': 0.08, 'y1': -0.54, 'x2': 0.02, 'y2': -0.49, 'x3': -0.09, 'y3': -0.55,
               'x4': 0.03, 'y4': -0.55, 'x5': -0.14, 'y5': -0.57}
    probe11 = {'x1': -0.16, 'y1': -0.57, 'x2': 0.00, 'y2': -0.52, 'x3': -0.11, 'y3': -0.57,
               'x4': -0.10, 'y4': -0.52, 'x5': -0.04, 'y5': -0.51}
    probe12 = {'x1': -0.14, 'y1': -0.53, 'x2': -0.06, 'y2': -0.51, 'x3': 0.01, 'y3': -0.53,
               'x4': -0.06, 'y4': -0.49, 'x5': -0.15, 'y5': -0.52}

    probes = [probe1, probe2, probe3, probe4, probe5, probe6, probe7, probe8, probe9, probe10, probe11, probe12]
    results = [10.6, 10.7, 11.0, 9.4, 10.3, 9.2, 11.2, 0, 0, 0, 0, 0]
    for j in range(12):
        optimizer.register(params=probes[j], target=results[j])

    for _ in range(100):
        state_list = []
        intersection_vec = []
        left_connect = False
        right_connect = False
        all_connect = False
        shape = []
        next_to_probe = optimizer.suggest(utility)  # A dict to tell you the next parameters to probe

        for item in next_to_probe:
            next_to_probe[item] = round(next_to_probe[item])
            state_list.append(next_to_probe[item])

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
            count += 1
            reward = 100

        else:
            reward = 0

        try:
            optimizer.register(
                params=next_to_probe,
                target=float(reward)
            )

        except KeyError:
            next_to_probe['x1'] = next_to_probe['x1'] + random.randint(1, 1000)*0.001
            optimizer.register(
                params=next_to_probe,
                target=float(reward)
            )

    print(count)

if __name__ == "__main__":
    circuitBot()
    print("Done")
    # print(a)

    # center = [[state_list[i] * 0.01 - 0.04, state_list[i + 5] * 0.01 - 0.53] for i in range(0, 5)]
    # center = [[state_list[i], state_list[i + 5]] for i in range(0, 5)]
    # dic = {"x1": center[0][0], 'y1': center[0][1], 'x2': center[1][0], 'y2': center[1][1],
    #        'x3': center[2][0], 'y3': center[2][1], 'x4': center[3][0], 'y4': center[3][1],
    #        'x5': center[4][0], 'y5': center[4][1]}
    # print(dic)