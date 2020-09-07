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

# from bayes_opt import SequentialDomainReductionTransformer

# Bound region of parameter space
# For now, we have 6 parameters. The range of x is (-0.21, 0.1), the range of y is (-0.55, -0.4)
xybounds = {'x1': (-0.15, 0.09), 'y1': (-0.6, -0.46), 'x2': (-0.15, 0.09), 'y2': (-0.6, -0.46), 'x3': (-0.15, 0.09),
            'y3': (-0.6, -0.46)}
# Times of experiment
times_of_experiments = 10


def circuitBot():
    optimizer = BayesianOptimization(
        f=None,
        pbounds=xybounds,
        verbose=2,  # choices: 0, 1, 2. verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
        random_state=1,
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
    csv_list = []

    # train part
    for i in range(20):
        next_to_probe = optimizer.suggest(utility)  # A dict to tell you the next parameters to probe
        print(next_to_probe)
        # 将连续值变为离散值
        for item in next_to_probe:
            next_to_probe[item] = round(next_to_probe[item], 2)
        point_file = open("next.txt", "w")
        point_str = str()
        for key in next_to_probe:
            print(key)
            print(next_to_probe[key])
            point_str += str(next_to_probe[key])
            point_str += " "
        print(point_str)
        point_file.write(point_str)
        point_file.close()

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

        try:
            optimizer.register(
                params=next_to_probe,
                target=float(voltage)
            )
        except KeyError:
            next_to_probe['x1'] = next_to_probe['x1'] + random.randint(1, 1000) * 0.001
            optimizer.register(
                params=next_to_probe,
                target=float(voltage)
            )
        # To record these parameter and voltage
        tmp = []
        for key in next_to_probe:
            tmp.append(next_to_probe[key])
        tmp.append(float(voltage))
        csv_list.append(tmp)

    csv = pd.DataFrame(columns=name, data=csv_list)
    output_file = "data.csv"
    csv.to_csv(output_file)


if __name__ == "__main__":
    circuitBot()