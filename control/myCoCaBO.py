from bayes_opt import BayesianOptimization
from bayes_opt import UtilityFunction
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
from bayes_opt.util import load_logs
import numpy as np
import random
from shapely.geometry import Point, LineString, Polygon, MultiLineString


class CoCaBO:
    def __init__(self, acq_fun='ucb'):
        self.acq_fun = acq_fun

        # Store the ht recommendations for each iteration
        self.ht_recommendations = []
        self.ht_hist_batch = []

        bounds = {'x1': (-0.16, 0.08), 'y1': (-0.58, -0.48), 'x2': (-0.16, 0.08), 'y2': (-0.58, -0.48),
                  'x3': (-0.16, 0.08), 'y3': (-0.58, -0.48), 'x4': (-0.16, 0.08), 'y4': (-0.58, -0.48),
                  'x5': (-0.16, 0.08), 'y5': (-0.58, -0.48)}

        self.optimizer = BayesianOptimization(
            f=None,
            pbounds=bounds,
            verbose=2,  # choices: 0, 1, 2. verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
            # random_state=int(random.random() * 100),
            random_state=1,
            # bounds_transformer = SequentialDomainReductionTransformer()
        )
        # For kind = "ucb", small kappa (1) prefer exploitation, big kappa (10) prefer exploration
        # For kind = "ei", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration
        # kind = "poi", small xi (0.0) prefer exploitation, big xi (0.0) prefer exploration

        self.logger = JSONLogger(path="./Logs.json")
        self.optimizer.subscribe(Events.OPTIMIZATION_STEP, self.logger)

        self.utility = UtilityFunction(kind="ucb", kappa=10, xi=0.0)


    def initialize(self):
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
            self.optimizer.register(params=probes[j], target=results[j])



