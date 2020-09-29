# -*- coding: utf-8 -*-
#==========================================
# Title:  run_cocabo_exps.py
# Author: Binxin Ru and Ahsan Alvi
# Date:   20 August 2019
# Link:   https://arxiv.org/abs/1906.08878
#==========================================

# =============================================================================
#  CoCaBO Algorithms 
# =============================================================================
import sys
# sys.path.append('../bayesopt')
# sys.path.append('../ml_utils')
import argparse
import os
from read_voltage import read_voltage_line_circle, read_voltage_simulation
from methods.CoCaBO import CoCaBO
from methods.BatchCoCaBO import BatchCoCaBO


def CoCaBO_Exps(obj_func, budget, initN=0, trials=1, kernel_mix=0.5, batch=1):

    # define saving path for saving the results
    saving_path = f'result/'
    if not os.path.exists(saving_path):
        os.makedirs(saving_path)

    # define the objective function

    if obj_func == 'voltage':
        f = read_voltage_line_circle
        f = read_voltage_simulation
        # categories = [6, 6, 6, 6, 6]
        categories = [2, 2, 2, 2, 2]

        # bounds = [{'name': 'h1', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'h2', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'h3', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'h4', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'h5', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'x1', 'type': 'continuous', 'domain': (-12, 12)},
        #     {'name': 'y1', 'type': 'continuous', 'domain': (-5, 5)},
        #     {'name': 'x2', 'type': 'continuous', 'domain': (-12, 12)},
        #     {'name': 'y2', 'type': 'continuous', 'domain': (-5, 5)},
        #     {'name': 'x3', 'type': 'continuous', 'domain': (-12, 12)},
        #     {'name': 'y3', 'type': 'continuous', 'domain': (-5, 5)},
        #     {'name': 'x4', 'type': 'continuous', 'domain': (-12, 12)},
        #     {'name': 'y4', 'type': 'continuous', 'domain': (-5, 5)},
        #     {'name': 'x5', 'type': 'continuous', 'domain': (-12, 12)},
        #     {'name': 'y5', 'type': 'continuous', 'domain': (-5, 5)}]

        # bounds = [{'name': 'h1', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'h2', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'h3', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'h4', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'h5', 'type': 'categorical', 'domain': (0, 1, 2, 3, 4, 5)},
        #     {'name': 'x1', 'type': 'continuous', 'domain': (-12, -9)},
        #     {'name': 'y1', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x2', 'type': 'continuous', 'domain': (-6, -2)},
        #     {'name': 'y2', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x3', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'y3', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x4', 'type': 'continuous', 'domain': (2, 6)},
        #     {'name': 'y4', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x5', 'type': 'continuous', 'domain': (9, 12)},
        #     {'name': 'y5', 'type': 'continuous', 'domain': (-2, 2)}]


        # bounds = [{'name': 'h1', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h2', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h3', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h4', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h5', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'x1', 'type': 'continuous', 'domain': (-12, -9)},
        #     {'name': 'y1', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x2', 'type': 'continuous', 'domain': (-6, -2)},
        #     {'name': 'y2', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x3', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'y3', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x4', 'type': 'continuous', 'domain': (2, 6)},
        #     {'name': 'y4', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x5', 'type': 'continuous', 'domain': (9, 12)},
        #     {'name': 'y5', 'type': 'continuous', 'domain': (-2, 2)}]


        # bounds = [{'name': 'h1', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h2', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h3', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h4', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h5', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'x1', 'type': 'continuous', 'domain': (-12, -10)},
        #     {'name': 'y1', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x2', 'type': 'continuous', 'domain': (-8, -4)},
        #     {'name': 'y2', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x3', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'y3', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x4', 'type': 'continuous', 'domain': (4, 8)},
        #     {'name': 'y4', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x5', 'type': 'continuous', 'domain': (10, 12)},
        #     {'name': 'y5', 'type': 'continuous', 'domain': (-2, 2)}]

        # init = -14 14

        # bounds = [{'name': 'h1', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h2', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h3', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h4', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'h5', 'type': 'categorical', 'domain': (0, 1)},
        #     {'name': 'x1', 'type': 'continuous', 'domain': (-8, -4)},
        #     {'name': 'y1', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x2', 'type': 'continuous', 'domain': (-2, -2)},
        #     {'name': 'y2', 'type': 'continuous', 'domain': (-2, 2)},
        #     {'name': 'x3', 'type': 'continuous', 'domain': (4, 8)},
        #     {'name': 'y3', 'type': 'continuous', 'domain': (-2, 2)}]

        # init = -14 14

        bounds = [{'name': 'h1', 'type': 'categorical', 'domain': (0, 1)},
            {'name': 'h2', 'type': 'categorical', 'domain': (0, 1)},
            {'name': 'h3', 'type': 'categorical', 'domain': (0, 1)},
            {'name': 'h4', 'type': 'categorical', 'domain': (0, 1)},
            {'name': 'h5', 'type': 'categorical', 'domain': (0, 1)},
            {'name': 'x1', 'type': 'continuous', 'domain': (-8, -4)},
            {'name': 'x2', 'type': 'continuous', 'domain': (-2, -2)},
            {'name': 'x3', 'type': 'continuous', 'domain': (4, 8)}]

    else:
        raise NotImplementedError

        
    # Run CoCaBO Algorithm
    if batch == 1:
        # sequential CoCaBO
        mabbo = CoCaBO(objfn=f, initN=initN, bounds=bounds,
                       acq_type='LCB', C=categories,
                       kernel_mix = kernel_mix)

    else:
        # batch CoCaBO
        mabbo = BatchCoCaBO(objfn=f, initN=initN, bounds=bounds,
                            acq_type='LCB', C=categories,
                            kernel_mix=kernel_mix,
                            batch_size=batch)
    mabbo.runTrials(trials, budget, saving_path)




if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Run BayesOpt Experiments")
    parser.add_argument('-f', '--func', help='Objective function',
                        default='voltage', type=str)
    parser.add_argument('-mix', '--kernel_mix',
                        help='Mixture weight for production and summation kernel. Default = 0.0', default=0.5,
                        type=float)
    parser.add_argument('-n', '--max_itr', help='Max Optimisation iterations. Default = 100',
                        default=20, type=int)
    parser.add_argument('-tl', '--trials', help='Number of random trials. Default = 20',
                        default=1, type=int)
    parser.add_argument('-b', '--batch', help='Batch size (>1 for batch CoCaBO and =1 for sequential CoCaBO). Default = 1',
                        default=1, type=int)

    args = parser.parse_args()
    # print(f"Got arguments: \n{args}")
    obj_func = args.func
    kernel_mix = args.kernel_mix
    n_itrs = args.max_itr
    n_trials = args.trials
    batch = args.batch

    CoCaBO_Exps(obj_func=obj_func, budget=n_itrs,
                 trials=n_trials, kernel_mix = kernel_mix, batch=batch)
