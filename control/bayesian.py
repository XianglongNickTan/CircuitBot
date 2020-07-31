# -*- coding: utf-8 -*-
import os
import time
import pandas as pd
from bayes_opt import BayesianOptimization
from bayes_opt import UtilityFunction
import keyboard
# from bayes_opt import SequentialDomainReductionTransformer


# from control import MoveItIkDemo



def circuitBot():
    optimizer = BayesianOptimization(
        f=None,
        pbounds={'x': (-0.3, 0.2), 'y': (-0.5, -0.2)},
        verbose=2,  # choices: 0, 1, 2
        random_state=1,
        # bounds_transformer = SequentialDomainReductionTransformer()
    )
    # For kind = "ucb", small kappa (1) prefer exploitation, big kappa (10) prefer exploration
    # For kind = "ei", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration
    # kind = "poi", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration

    utility = UtilityFunction(kind="ucb", kappa=10, xi=0.0)
    
    # To output paramters and voltage as a csv file
    name = ['x', 'y', 'voltage']
    csv_list = []

    # train part
    for _ in range(10):
        next_to_probe = optimizer.suggest(utility) # A dict to tell you the next parameters to probe

        point_file = open("point.txt", "w")
        point_str = str()
        for key in next_to_probe:
            point_str += next_to_probe[key]
            point_str += " "
        point_file.write(point_str)

        ###################################################
        # Now we should wait the robot arm draw circle
        # Pass q when the robot arm finish
        ###################################################

        keyboard.wait("q")

        voltage_file = open("voltage.txt", "r")
        voltage = voltage_file.read()
        voltage_file.close()
        os.remove("voltage.txt")

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

    csv = pd.DataFrame(columns = name, data = csv_list)
    output_file = "data.csv"
    csv.to_csv(output_file)



if __name__ == "__main__":
    circuitBot()