from bayes_opt import BayesianOptimization
from bayes_opt import UtilityFunction
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
from bayes_opt.util import load_logs
import numpy as np
import random
import math
import tqdm
from shapely.geometry import Point, LineString, Polygon, MultiLineString


from sklearn.gaussian_process.kernels import Matern
from sklearn.gaussian_process import GaussianProcessRegressor



class CoCaBO:
    def __init__(self, acq_fun='ucb'):
        self.co_dim =                   # dimension of continue variables
        self.ca_dim =                   # dimension of category variables
        self.num_ca = 6                  # number of category variables
        self.ca_list = ['hor_line', 'ver_line0', 'cross', 'circle', 'triangle', 'square']   # choices of category variables
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
        raise NotImplementedError

    def run_optim(self, max_iters, batch_size=1):

            # Initialize wts and probs
            b = batch_size
            bestUpperBoundEstimate = 2 * max_iters / 3
            gamma_list = [math.sqrt(self.num_ca * math.log(self.num_ca) / ((math.e - 1) * bestUpperBoundEstimate))] * self.ca_dim
            ca_weight_list = [np.ones(self.num_ca)] * self.ca_dim

            for t in tqdm(range(max_iters)):
                self.iteration = t

                # Compute the probability for each category and Choose categorical variables
                ht_list, prob_dist_list = self.compute_prob_dist_and_draw_hts(ca_weight_list, gamma_list, batch_size)

                # Get reward for multi-armed bandit
                reward = self.


                Gt_ht_list = self.RewardperCategoryviaBO(self.f, ht_list, self.ca_dim, self.co_dim,
                                                         categorical_dims,
                                                         continuous_dims,
                                                         self.bounds,
                                                         self.acq_type, b)

                # Update the reward and the weight
                Wc_list = self.update_weights_for_all_cat_var(
                    Gt_ht_list, ht_list,
                    Wc_list, gamma_list,
                    probabilityDistribution_list,
                    batch_size)

                # Get the best value till now
                besty, li, vi = self.getBestVal2(self.result)

                # Store the results of this iteration
                result_list.append([t, ht_list, Gt_ht_list, besty, self.mix_used,
                                    self.model_hp])

                self.ht_recommedations.append(ht_list)

            df = pd.DataFrame(result_list, columns=["iter", "ht", "Reward",
                                                    "best_value", "mix_val",
                                                    "model_hp"])
            bestx = self.data[li][vi]
            self.best_val_list.append([batch_size, self.trial_num, li, besty,
                                       bestx])

            return df






    def compute_prob_dist_and_draw_hts(self, ca_weight_list, gamma_list, batch_size):
        # if batch_size > 1:
        #     ht_batch_list = np.zeros((batch_size, len(self.ca_dim)))
        #     prob_dist_list = []
        #
        #     for j in range(len(self.ca_dim)):
        #         ca_weight = ca_weight_list[j]
        #         gamma = gamma_list[j]
        #         # perform some truncation here
        #         max_weight = np.max(ca_weight)
        #         temp = np.sum(ca_weight) * (1.0 / batch_size - gamma / self.num_ca) / (1 - gamma)
        #         if gamma < 1 and max_weight >= temp:
        #             # find a threshold alpha
        #             alpha = self.estimate_alpha(batch_size, gamma, ca_weight, self.num_ca)
        #             S0 = [idx for idx, val in enumerate(ca_weight) if val > alpha]
        #         else:
        #             S0 = []
        #         # Compute the probability for each category
        #         prob_dist = distr(ca_weight, gamma)
        #
        #         # draw a batch here
        #         if batch_size < len(self.ca_list):
        #             mybatch_ht = DepRound(prob_dist, k=batch_size)
        #         else:
        #             mybatch_ht = np.random.choice(len(prob_dist), batch_size, p=prob_dist)
        #
        #         # ht_batch_list size: len(self.C_list) x B
        #         ht_batch_list[:, j] = mybatch_ht[:]
        #
        #         # ht_batch_list.append(mybatch_ht)
        #         prob_dist_list.append(prob_dist)
        #
        #     return ht_batch_list, prob_dist_list, S0
        #
        # else:
        ht_list = []
        prob_dist_list = []
        for j in range(self.ca_dim):
            ca_weight = ca_weight_list[j]
            gamma = gamma_list[j]
            # Compute the probability for each category
            prob_dist = distr(ca_weight, gamma)
            # Choose a categorical variable at random
            ht = draw(prob_dist)
            ht_list.append(ht)
            prob_dist_list.append(prob_dist)

        return ht_list, prob_dist_list




    def update_weights_for_all_cat_var(self, reward,  Gt_ht_list, ht_batch_list, ca_weight_list, gamma_list,
                                       prob_dist_list, batch_size, S0=None):
        for j in range(self.ca_dim):
            ca_weight = ca_weight_list[j]
            gamma = gamma_list[j]
            prob_dist = prob_dist_list
            #
            # if batch_size > 1:
            #     ht_batch_list = ht_batch_list.astype(int)
            #     Gt_ht = Gt_ht_list[:, j]
            #     mybatch_ht = ht_batch_list[:, j]  # 1xB
            #     for ii, ht in enumerate(mybatch_ht):
            #         Gt_ht_b = Gt_ht[ii]
            #         estimatedReward = 1.0 * Gt_ht_b / probabilityDistribution[ht]
            #         if ht not in S0:
            #             Wc[ht] *= np.exp(batch_size * estimatedReward * gamma / C)
            # else:
            reward = reward_list[j]
            Gt_ht = Gt_ht_list[j]
            ht = ht_batch_list[j]  # 1xB
            estimatedReward = 1.0 * Gt_ht / probabilityDistribution[ht]
            Wc[ht] *= np.exp(estimatedReward * gamma / C)

        return Wc_list





    def get_kernel(self, categorical_dims, continuous_dims):
            # create surrogate
            if self.ARD:
                hp_bounds = np.array([
                    *[[1e-4, 3]] * len(continuous_dims),  # cont lengthscale
                    [1e-6, 1],  # likelihood variance
                ])
            else:
                hp_bounds = np.array([
                    [1e-4, 3],  # cont lengthscale
                    [1e-6, 1],  # likelihood variance
                ])
            fix_mix_in_this_iter, mix_value, hp_bounds = self.get_mix(hp_bounds)
            k_cat = CategoryOverlapKernel(len(categorical_dims),
                                        active_dims=categorical_dims)  # cat
            k_cont = GPy.kern.Matern52(len(continuous_dims),
                                    lengthscale=self.default_cont_lengthscale,
                                    active_dims=continuous_dims,
                                    ARD=self.ARD)  # cont
            my_kernel = MixtureViaSumAndProduct(
                len(categorical_dims) + len(continuous_dims),
                k_cat, k_cont, mix=mix_value, fix_inner_variances=True,
                fix_mix=fix_mix_in_this_iter)
            return my_kernel, hp_bounds





















# distr: [float] -> (float)
# Normalize a list of floats to a probability distribution.  Gamma is an
# egalitarianism factor, which tempers the distribtuion toward being uniform as
# it grows from zero to one.
def distr(weights, gamma=0.0):
    theSum = float(sum(weights))
    return tuple((1.0 - gamma) * (w / theSum) + (gamma / len(weights)) for w in weights)



# draw: [float] -> int
# pick an index from the given list of floats proportionally
# to the size of the entry (i.e. normalize to a probability
# distribution and draw according to the probabilities).
def draw(weights):
    choice = random.uniform(0, sum(weights))
    #    print(choice)
    choiceIndex = 0

    for weight in weights:
        choice -= weight
        if choice <= 0:
            return choiceIndex
        choiceIndex += 1
