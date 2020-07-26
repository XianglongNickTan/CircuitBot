# -*- coding: utf-8 -*-
import os
import time
import pandas as pd
from bayes_opt import BayesianOptimization
from bayes_opt import UtilityFunction

from black_box_function import MoveItIkDemo



def circuitBot(robot_arm):
    optimizer = BayesianOptimization(
        f=None,
        pbounds={'x': (-100, 100), 'y': (-70, 70)},
        verbose=2,
        random_state=1,
    )

    utility = UtilityFunction(kind="ucb", kappa=2.5, xi=0.0)

    name = ['x', 'y', 'voltage']
    csv_list = []

    # train part
    for _ in range(10):
        next_to_probe = optimizer.suggest(utility) # A dict to tell you the next parameters to probe

        robot_arm.draw_circle(next_to_probe)

        #time.sleep(1)

        voltage_file = open("voltage.txt", "r")
        # while not voltage_file:
        #     print(voltage_file)
        #     voltage_file = open("voltage.txt", "r")
        voltage = voltage_file.read()
        voltage_file.close()
        os.remove("voltage.txt")
        print(voltage)

        optimizer.register(
            params=next_to_probe,
            target=float(voltage)
        )

        tmp = []
        for key in next_to_probe:
            tmp.append(next_to_probe[key])
        tmp.append(float(voltage))
        csv_list.append(tmp)

    csv = pd.DataFrame(columns = name, data = csv_list)
    output_file = "data.csv"
    csv.to_csv(output_file)



if __name__ == "__main__":
    demo = MoveItIkDemo()
    circuitBot(demo)