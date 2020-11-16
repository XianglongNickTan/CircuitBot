# -*- coding: utf-8 -*-
#==========================================
# Title:  CoCaBO.py
# Author: Binxin Ru and Ahsan Alvi
# Date:   20 August 2019
# Link:   https://arxiv.org/abs/1906.08878
#==========================================

import math

import GPy
import numpy as np
import pandas as pd
from tqdm import tqdm
import random
import pickle

from utils.bayesopt.acquisition import AcquisitionOnSubspace, EI, UCB
from methods.CoCaBO_Base import CoCaBO_Base
from utils.ml_utils.models import GP
from utils.ml_utils.models.additive_gp import MixtureViaSumAndProduct, CategoryOverlapKernel
from utils.ml_utils.optimization import sample_then_minimize
from utils.connection import check_connection

''' Sequential CoCaBO algorithm '''
class CoCaBO(CoCaBO_Base):

    def __init__(self, objfn, initN, bounds, acq_type, C, **kwargs):

        super(CoCaBO, self).__init__(objfn, initN, bounds, acq_type, C, **kwargs)
        self.best_val_list = []
        self.C_list = self.C
        self.name = 'CircuitBot'
        self.exp_data = None

    def runOptim(self, budget, seed, batch_size=1, initData=None, initResult=None):

        if (initData and initResult):
            self.data = initData[:]
            self.result = initResult[:]
        else:  # We do not need this
            # self.data = [np.array([[]])]
            # self.result = [np.array([[]])]
            self.data, self.result = self.initialise(seed)

        # Initialize wts and probs
        b = batch_size
        bestUpperBoundEstimate = 2 * budget / 3
        gamma_list = [math.sqrt(C * math.log(C) /
                                ((math.e - 1) * bestUpperBoundEstimate))
                      for C in self.C_list]

        #####################
        # gamma_list = [0.1555754826798944 for C in self.C_list]
        #####################


        Wc_list_init = [np.ones(C) for C in self.C_list]

        ###########
        # Wc_list_init = [np.array([1, 2.2897309854472105]),
        #                 np.array([1, 4.7361019048697415]),
        #                 np.array([1, 73.17207652394148]),
        #                 np.array([1, 7.710164430018213]),
        #                 np.array([1, 3.471682642581374])]
        ###########

        Wc_list = Wc_list_init
        nDim = len(self.bounds)

        result_list = []

        continuous_dims = list(range(len(self.C_list), nDim))
        categorical_dims = list(range(len(self.C_list)))

        ################### import data draw prob ##############
        # data_fname = f'data/experiment_mean_no_barrier'
        #
        # with open(data_fname, 'rb') as init_data_file:
        #     self.exp_data = pickle.load(init_data_file)

        ########################################################




        for t in tqdm(range(budget)):
            circle_prob = []
            self.iteration = t

            # Compute the probability for each category and Choose categorical variables
            ht_list, probabilityDistribution_list = \
                self.compute_prob_dist_and_draw_hts(Wc_list, gamma_list,
                                                    batch_size)

            # for i in range(5):
            #     prob = probabilityDistribution_list[i][1]
            #     circle_prob.append(prob)

            # ht_list = self.exp_data.ht[self.iteration]

            p1 = probabilityDistribution_list[0][1]
            p2 = probabilityDistribution_list[1][1]
            p3 = probabilityDistribution_list[2][1]
            p4 = probabilityDistribution_list[3][1]
            p5 = probabilityDistribution_list[4][1]
            # circle_prob.append(prob)


            # Get reward for multi-armed bandit
            Gt_ht_list, x_list, z_list, y_list = self.RewardperCategoryviaBO(self.f, ht_list,
                                                     categorical_dims,
                                                     continuous_dims,
                                                     self.bounds,
                                                     self.acq_type, b)

            line = float(ht_list.count(0))
            circle = float(ht_list.count(1))

            # Update the reward and the weight
            Wc_list = self.update_weights_for_all_cat_var(
                Gt_ht_list, ht_list,
                Wc_list, gamma_list,
                probabilityDistribution_list,
                batch_size)

            # Get the best value till now
            besty, li, vi = self.getBestVal2(self.result)

            if self.use_mean:
                besty = (besty * (self.std_y + 1e-10) + self.mean_y) / self.reward_scale

            resistance = (30 - y_list) / y_list * 45

            # Store the results of this iteration
            # result_list.append([ht_list, z_list, circle, y_list, resistance, circle_prob, besty,
            #                     self.model_hp])
            result_list.append([ht_list, z_list, circle, y_list, resistance, p1, p2, p3, p4, p5, besty,
                                self.model_hp])
            self.ht_recommedations.append(ht_list)
            if self.print_bestval:
                print(f'y_best = {besty}')
            if self.print_weight:
                print(Wc_list)


        df = pd.DataFrame(result_list, columns=["ht", "zt", "circle_num", "voltage", "resistance", "p1", "p2", "p3", "p4", "p5",
                                                "best_value", "model_hp"])
        bestx = self.data[li][vi]
        self.best_val_list.append([batch_size, self.trial_num, li, besty,
                                   bestx])



        return df

    # =============================================================================
    #   Function returns the reward for multi-armed bandit
    # =============================================================================
    def RewardperCategoryviaBO(self, objfn, ht_next_list, categorical_dims,
                               continuous_dims, bounds, acq_type, b):

        #  Get observation data

        Zt = self.data[0]
        yt = self.result[0]

        my_kernel, hp_bounds = self.get_kernel(categorical_dims,
                                               continuous_dims)

        gp_opt_params = {'method': 'multigrad',
                         'num_restarts': 5,
                         'restart_bounds': hp_bounds,
                         'hp_bounds': hp_bounds,
                         'verbose': False}

        gp = GP(Zt, yt, my_kernel, y_norm='meanstd',
                opt_params=gp_opt_params)

        opt_flag, gp = self.set_model_params_and_opt_flag(gp)
        if opt_flag:
            # print("\noptimising!\n")
            gp.optimize()
        self.model_hp = gp.param_array


        self.mix_used = gp.kern.mix[0]

        x_bounds = np.array([d['domain'] for d in bounds
                             if d['type'] == 'continuous'])
        # create acq
        if acq_type == 'EI':
            acq = EI(gp, np.min(gp.Y_raw))
        elif acq_type == 'LCB':
            acq = UCB(gp, 2)

        acq_sub = AcquisitionOnSubspace(acq, my_kernel.k2.active_dims,
                                        ht_next_list)

        def optimiser_func(x):
            # return -acq_sub.evaluate(np.atleast_2d(x))
            return acq_sub.evaluate(np.atleast_2d(x))

        res = sample_then_minimize(
            optimiser_func,
            x_bounds,
            num_samples=20,
            num_chunks=5,
            num_local=2,
            evaluate_sequentially=False)

        x_next = res.x
        z_next = np.hstack((ht_next_list, x_next))

        if self.print_arm:
            print(f'arm pulled={ht_next_list[:]}')


        #  Evaluate objective function at z_next = [x_next,  ht_next_list]
        y_next = objfn(z_next)

        # y_next = self.exp_data.voltage[self.iteration]


        y_next *= self.reward_scale

        # Append recommeded data

        self.data[0] = np.row_stack((self.data[0], z_next))

        if self.use_mean:
            y_next = (y_next - self.mean_y) / (self.std_y + 1e-10)
            self.result[0] = np.row_stack((self.result[0], y_next))
            ########### change back to normalised before #############
            y_next = (y_next * (self.std_y + 1e-10) + self.mean_y) / self.reward_scale

        else:
            self.result[0] = np.row_stack((self.result[0], y_next))
            y_next /= self.reward_scale


        # Obtain the reward for each categorical variable
        ht_next_list_array = np.atleast_2d(ht_next_list)
        ht_list_rewards = self.compute_reward_for_all_cat_variable(
            ht_next_list_array, b)
        ht_list_rewards = list(ht_list_rewards.flatten())

        # bestval_ht = np.max(self.result[0] * -1)
        bestval_ht = np.max(self.result[0])

        if self.print_reward:
            print(f'rewards = {ht_list_rewards[:]}')

        return ht_list_rewards, x_next, z_next, y_next

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
