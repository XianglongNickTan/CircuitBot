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
import pickle
from read_voltage import read_voltage


# from bayes_opt import SequentialDomainReductionTransformer

# Bound region of parameter space
# For now, we have 6 parameters. The range of x is (-0.21, 0.1), the range of y is (-0.55, -0.4)

bounds = [{'name': 'h1', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'h2', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'h3', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'h4', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'h5', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'x1', 'type': 'continuous', 'domain': (-12, 12)},
          {'name': 'y1', 'type': 'continuous', 'domain': (-5, 5)},
          {'name': 'x2', 'type': 'continuous', 'domain': (-12, 12)},
          {'name': 'y2', 'type': 'continuous', 'domain': (-5, 5)},
          {'name': 'x3', 'type': 'continuous', 'domain': (-12, 12)},
          {'name': 'y3', 'type': 'continuous', 'domain': (-5, 5)},
          {'name': 'x4', 'type': 'continuous', 'domain': (-12, 12)},
          {'name': 'y4', 'type': 'continuous', 'domain': (-5, 5)},
          {'name': 'x5', 'type': 'continuous', 'domain': (-12, 12)},
          {'name': 'y5', 'type': 'continuous', 'domain': (-5, 5)}]

bounds = [{'name': 'h1', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'h2', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'h3', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'h4', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'h5', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
          {'name': 'x1', 'type': 'continuous', 'domain': (-12, -7)},
          {'name': 'y1', 'type': 'continuous', 'domain': (-5, 5)},
          {'name': 'x2', 'type': 'continuous', 'domain': (-9, -3)},
          {'name': 'y2', 'type': 'continuous', 'domain': (-5, 5)},
          {'name': 'x3', 'type': 'continuous', 'domain': (-3, 3)},
          {'name': 'y3', 'type': 'continuous', 'domain': (-5, 5)},
          {'name': 'x4', 'type': 'continuous', 'domain': (3, 9)},
          {'name': 'y4', 'type': 'continuous', 'domain': (-5, 5)},
          {'name': 'x5', 'type': 'continuous', 'domain': (7, 12)},
          {'name': 'y5', 'type': 'continuous', 'domain': (-5, 5)}]

def initialise(seed, saving_path, C, initN, bounds, f):
    """Get NxN intial points"""
    data = []
    result = []

    np.random.seed(seed)
    random.seed(seed)

    init_fname = saving_path + 'init_data_' + str(seed)

    # if os.path.exists(init_fname):
    #     print(f"Using existing init data for seed {seed}")
    #     with open(init_fname, 'rb') as init_data_filefile2:
    #         init_data = pickle.load(init_data_filefile2)
    #     Zinit = init_data['Z_init']
    #     yinit = init_data['y_init']
    if True:
        print(f"Creating init data for seed {seed}")
        Xinit = generateInitialPoints(initN,
                                           bounds[len(C):])
        hinit = np.hstack(
            [np.random.randint(0, s, initN)[:, None] for s in C])
        Zinit = np.hstack((hinit, Xinit))
        yinit = np.zeros([Zinit.shape[0], 1])

        for j in range(initN):
            ht_list = list(hinit[j])
            yinit[j] = f(np.hstack((ht_list, Xinit[j])))
        # print(ht_list, Xinit[j], yinit[j])

        init_data = {}
        init_data['Z_init'] = Zinit
        init_data['y_init'] = yinit

        with open(init_fname, 'wb') as init_data_file:
            pickle.dump(init_data, init_data_file)

    data.append(Zinit)
    result.append(yinit)
    return data, result


def generateInitialPoints(initN, bounds):
    nDim = len(bounds)
    Xinit = np.zeros((initN, len(bounds)))
    for i in range(initN):
        Xinit[i, :] = np.array(
            [np.random.uniform(bounds[b]['domain'][0],
                               bounds[b]['domain'][1], 1)[0]
             for b in range(nDim)])
    return Xinit


saving_path = f'data/'
categories = [6, 6, 6, 6, 6]
function = read_voltage
data, result = initialise(108, saving_path, categories, 50, bounds, function)
print(data)

# if __name__ == "__main__":
#     circuitBot()
#     print("Done")
#     # print(a)
