# -*- coding: utf-8 -*-
#==========================================
# Title:  CoCaBO_Base.py
# Author: Binxin Ru and Ahsan Alvi
# Date:   20 August 2019
# Link:   https://arxiv.org/abs/1906.08878
#==========================================

import collections
import pickle
import random

import numpy as np
from scipy.optimize import minimize

from methods.BaseBO import BaseBO
from utils.DepRound import DepRound
from utils.probability import distr, draw
from collections import Counter


class CoCaBO_Base(BaseBO):

    def __init__(self, objfn, initN, bounds, acq_type, C,
                 kernel_mix=0.5, mix_lr=10,
                 model_update_interval=1,
                 ard=False, **kwargs):
        super().__init__(objfn, initN, bounds, C, **kwargs)
        self.acq_type = acq_type

        # Store the ht recommendations for each iteration
        self.ht_recommedations = []
        self.ht_hist_batch = []

        # Store the name of the algorithm
        self.policy = None

        self.X = []
        self.Y = []

        # To check the best vals
        self.gp_bestvals = []

        self.ARD = ard

        # Keeping track of current iteration helps control mix learning
        self.iteration = None

        self.model_hp = None
        self.default_cont_lengthscale = 0.2

        self.mix = kernel_mix
        if ((model_update_interval % mix_lr == 0) or
                (mix_lr % model_update_interval == 0)):
            self.mix_learn_rate = mix_lr
            self.model_update_interval = model_update_interval
        else:
            self.mix_learn_rate = min(mix_lr, model_update_interval)
            self.model_update_interval = min(mix_lr, model_update_interval)
        self.mix_used = 0.5

        self.name = None

    def estimate_alpha(self, batch_size, gamma, Wc, C):

        def single_evaluation(alpha):
            denominator = sum([alpha if val > alpha else val for idx, val in enumerate(Wc)])
            rightside = (1 / batch_size - gamma / C) / (1 - gamma)
            output = np.abs(alpha / denominator - rightside)

            return output

        x_tries = np.random.uniform(0, np.max(Wc), size=(100, 1))
        y_tries = [single_evaluation(val) for val in x_tries]
        # find x optimal for init
        # print(f'ytry_len={len(y_tries)}')
        idx_min = np.argmin(y_tries)
        x_init_min = x_tries[idx_min]

        res = minimize(single_evaluation, x_init_min, method='BFGS', options={'gtol': 1e-6, 'disp': False})
        if isinstance(res, float):
            return res
        else:
            return res.x

    def runTrials(self, trials, budget, saving_path):
        # Initialize mean_bestvals, stderr, hist
        best_vals = []
        mix_values = []
        debug_values = []
        n_working = trials
        self.saving_path = saving_path
        np.random.seed(108)
        random.seed(108)

        for i in range(trials):
            # print("Running trial: ", i)
            self.trial_num = 1
            # np.random.seed(i)
            # random.seed(i)
            #
            # probe1 = [2, 2, 5, 1, 2, 8, 1, -11, 1, 3, -3, -9, -3, -7, 2]
            # probe2 = [2, 2, 4, 3, 3, 6, 3, -2, 2, -8, -4, -9, 4, 9, -2]
            # probe3 = [2, 4, 4, 2, 3, 8, 3, -1, 3, -10, -2, -5, -2, 3, 4]
            # probe4 = [4, 5, 2, 1, 5, -5, -4, -2, -3, -10, 0, -7, -3, 7, -2]
            # probe5 = [4, 3, 1, 0, 2, -8, 5, 5, 4, 7, 1, 12, -2, -3, 3]
            # probe6 = [5, 4, 3, 3, 4, 11, -3, 6, -5, -9, -1, 3, 2, 0, -1]
            # probe7 = [3, 3, 2, 4, 0, 11, -4, -8, 0, 1, 0, -2, -2, 8, -2]
            # probe8 = [3, 4, 3, 0, 2, 8, 3, -8, 4, -3, 2, -1, -4, 5, 5]
            # probe9 = [2, 3, 3, 3, 3, 6, -1, -8, 1, 8, 3, 0, 4, -6, 4]
            # probe10 = [5, 2, 2, 2, 5, -1, 3, -12, -1, 9, 2, -5, -3, 11, 3]
            # probe11 = [3, 0, 3, 4, 4, -11, 2, -7, 3, 9, 3, 1, 4, 7, -2]
            # probe12 = [5, 4, 2, 3, 3, 7, 1, 3, 1, -5, 4, -7, -3, -11, 0]
            # probe13 = [3, 2, 2, 4, 5, 9, 0, -6, -2, 2, 0, 1, 2, -7, -3]
            # probe14 = [5, 2, 3, 3, 2, -2, 5, 3, -1, 5, 1, -10, 4, 12, 2]
            # probe15 = [1, 3, 3, 5, 0, 1, -5, -2, 0, -10, 2, 8, -4, -2, -3]
            # probe16 = [2, 4, 3, 3, 2, -8, 2, -4, 1, 3, -2, 5, 5, 9, 0]
            # probe17 = [5, 2, 4, 4, 4, 9, -5, -12, -5, -10, -4, 2, 0, -5, 2]
            # probe18 = [5, 3, 1, 0, 4, 8, 0, -8, 0, 8, 3, -6, 4, 1, 3]
            # probe19 = [4, 4, 0, 4, 5, -8, 3, 0, -2, -1, 2, -10, 1, 8, 0]
            # probe20 = [3, 1, 3, 3, 0, -12, -1, -7, -1, 8, -2, 0, -3, -7, 0]
            # initData = [probe1, probe2, probe3, probe4, probe5, probe6, probe7, probe8, probe9, probe10,
            #     probe11, probe12, probe13, probe14, probe15, probe16, probe17, probe18, probe19, probe20]
            # initData = [np.array(initData)]

            initData = [[0.0, 0.0, 0.0, 1.0, 1.0, -6.33734928, 1.58121456, -6.80855969]]
            initData = [np.array(initData)]

            # initResult = [13.2, 16.5, 18.9, 18.9, 13.9, 18.5, 18.4, 17.3, 17.7, 18.2,
            #     17.5, 18.5, 19.9, 18.0, 18.1, 18.8, 18.8, 18.5, 20.2, 16.3]

            initResult = [0]

            for i in range(len(initResult)):
                initResult[i] = [-initResult[i]]

            initResult = [np.array(initResult)]

            df = self.runOptim(budget=budget, seed=None, initData=initData, initResult=initResult)
            # df = self.runOptim(budget=budget, seed=None, initData=None, initResult=None)
            # best_vals.append(df['best_value'])
            # mix_values.append(df['model_hp'])
            self.save_progress_to_disk(best_vals, debug_values, mix_values,
                                       saving_path, df)

        # Runoptim updates the ht_recommendation histogram
        self.best_vals = best_vals
        ht_hist = collections.Counter(np.array(self.ht_recommedations).ravel())
        self.ht_recommedations = []
        self.mean_best_vals = np.mean(best_vals, axis=0)
        self.err_best_vals = np.std(best_vals, axis=0) / np.sqrt(n_working)

        # For debugging
        self.gp_bestvals = best_vals
        self.ht_hist = ht_hist
        self.n_working = n_working
        return self.mean_best_vals, self.err_best_vals, ht_hist

    def save_progress_to_disk(self, best_vals, debug_values, mix_values,
                              saving_path, df):
        # results_file_name = saving_path + self.name + \
        #                     f'_{self.batch_size}' + \
        #                     '_best_vals_' + \
        #                     self.acq_type + \
        #                     '_ARD_' + str(self.ARD) + '_mix_' + \
        #                     str(self.mix)

        results_file_name = saving_path + self.name + \
                            f'_{self.batch_size}' + \
                            '_best_vals_' + \
                            self.acq_type + \
                            '_ARD_' + str(self.ARD) + '_mix_' + \
                            str(self.mix)

        with open(results_file_name, 'wb') as file:
            pickle.dump(best_vals, file)
        if self.mix > 1 or self.mix < 0:
            mix_file_name = saving_path + self.name + \
                            f'_{self.batch_size}_' + \
                            self.acq_type + \
                            '_ARD_' + str(self.ARD) + '_mix_' + \
                            str(self.mix) + '_mix_values'
            with open(mix_file_name, 'wb') as file2:
                pickle.dump(mix_values, file2)
        if self.debug:
            debug_file_name = saving_path + self.name + \
                              f'_{self.batch_size}_' + \
                              self.acq_type + \
                              '_ARD_' + str(self.ARD) + '_mix_' + \
                              str(self.mix) + '_debug'
            with open(debug_file_name, 'wb') as file2:
                pickle.dump(debug_values, file2)

        df.to_pickle(f"{results_file_name}_all")
        # df.to_pickle(f"{results_file_name}")


    def compute_reward_for_all_cat_variable(self, ht_next_batch_list, batch_size):
        # Obtain the reward for each categorical variable: B x len(self.C_list)
        ht_batch_list_rewards = np.zeros((batch_size, len(self.C_list)))
        for b in range(batch_size):
            ht_next_list = ht_next_batch_list[b, :]

            for i in range(len(ht_next_list)):
                idices = np.where(self.data[0][:, i] == ht_next_list[i])
                ht_result = self.result[0][idices]
                ht_reward = np.max(ht_result * -1)
                ht_batch_list_rewards[b, i] = ht_reward
        return ht_batch_list_rewards

    def update_weights_for_all_cat_var(self, Gt_ht_list, ht_batch_list, Wc_list, gamma_list,
                                       probabilityDistribution_list, batch_size, S0=None):
        for j in range(len(self.C_list)):
            Wc = Wc_list[j]
            C = self.C_list[j]
            gamma = gamma_list[j]
            probabilityDistribution = probabilityDistribution_list[j]
            # print(f'cat_var={j}, prob={probabilityDistribution}')

            if batch_size > 1:
                ht_batch_list = ht_batch_list.astype(int)
                Gt_ht = Gt_ht_list[:, j]
                mybatch_ht = ht_batch_list[:, j]  # 1xB
                for ii, ht in enumerate(mybatch_ht):
                    Gt_ht_b = Gt_ht[ii]
                    estimatedReward = 1.0 * Gt_ht_b / probabilityDistribution[ht]
                    if ht not in S0:
                        Wc[ht] *= np.exp(batch_size * estimatedReward * gamma / C)
            else:
                Gt_ht = Gt_ht_list[j]
                ht = ht_batch_list[j]  # 1xB
                estimatedReward = 1.0 * Gt_ht / probabilityDistribution[ht]
                Wc[ht] *= np.exp(estimatedReward * gamma / C)

        return Wc_list

    def compute_prob_dist_and_draw_hts(self, Wc_list, gamma_list, batch_size):

        if batch_size > 1:
            ht_batch_list = np.zeros((batch_size, len(self.C_list)))
            probabilityDistribution_list = []

            for j in range(len(self.C_list)):
                Wc = Wc_list[j]
                gamma = gamma_list[j]
                C = self.C_list[j]
                # perform some truncation here
                maxW = np.max(Wc)
                temp = np.sum(Wc) * (1.0 / batch_size - gamma / C) / (1 - gamma)
                if gamma < 1 and maxW >= temp:
                    # find a threshold alpha
                    alpha = self.estimate_alpha(batch_size, gamma, Wc, C)
                    S0 = [idx for idx, val in enumerate(Wc) if val > alpha]
                else:
                    S0 = []
                # Compute the probability for each category
                probabilityDistribution = distr(Wc, gamma)

                # draw a batch here
                if batch_size < C:
                    mybatch_ht = DepRound(probabilityDistribution, k=batch_size)
                else:
                    mybatch_ht = np.random.choice(len(probabilityDistribution), batch_size, p=probabilityDistribution)

                # ht_batch_list size: len(self.C_list) x B
                ht_batch_list[:, j] = mybatch_ht[:]

                # ht_batch_list.append(mybatch_ht)
                probabilityDistribution_list.append(probabilityDistribution)

            return ht_batch_list, probabilityDistribution_list, S0

        else:
            ht_list = []
            probabilityDistribution_list = []
            for j in range(len(self.C_list)):
                Wc = Wc_list[j]
                gamma = gamma_list[j]
                # Compute the probability for each category
                probabilityDistribution = distr(Wc, gamma)
                # Choose a categorical variable at random
                ht = draw(probabilityDistribution)
                ht_list.append(ht)
                probabilityDistribution_list.append(probabilityDistribution)

            return ht_list, probabilityDistribution_list

    def compute_weights_for_init_data(self, Wc_list_init, gamma_list, batch_size):
        ht_next_batch_list = self.data[0][:, :len(self.C)]
        _, probabilityDistribution_list, S0 = self.compute_prob_dist_and_draw_hts(Wc_list_init, gamma_list,
                                                                                  ht_next_batch_list.shape[0])
        Gt_ht_list = self.compute_reward_for_all_cat_variable(ht_next_batch_list, ht_next_batch_list.shape[0])
        New_Wc_list = self.update_weights_for_all_cat_var(Gt_ht_list, ht_next_batch_list, Wc_list_init, gamma_list,
                                                          probabilityDistribution_list, ht_next_batch_list.shape[0],
                                                          S0=S0)

        return New_Wc_list

    def get_mix(self, hp_bounds):
        fix_mix_in_this_iter = True
        if (self.mix >= 0) and (self.mix <= 1):  # mix param is fixed
            mix_value = self.mix
        elif ((self.iteration >= self.mix_learn_rate) and
              (self.iteration % self.mix_learn_rate == 0)):
            # learn mix
            hp_bounds = np.vstack(([1e-6, 1], hp_bounds))
            fix_mix_in_this_iter = False
            mix_value = 0.5
        else:  # between learning iterations
            mix_value = self.mix_used
        return fix_mix_in_this_iter, mix_value, hp_bounds

    # ========================================
    #     Over-ride this!
    # =============================================================================
    def runOptim(self, budget, seed):
        raise NotImplementedError

    # =============================================================================
    # Get best value from nested list along with the index
    # =============================================================================
    def getBestVal2(self, my_list):
        temp = [np.max(i * -1) for i in my_list]
        indx1 = [np.argmax(i * -1) for i in my_list]
        indx2 = np.argmax(temp)
        val = np.max(temp)
        list_indx = indx2
        val_indx = indx1[indx2]
        return val, list_indx, val_indx

    def set_model_params_and_opt_flag(self, model):
        """
        Returns opt_flag, model
        """
        if ((self.iteration >= self.model_update_interval) and
                (self.iteration % self.model_update_interval == 0)):
            return True, model
        else:
            # No previous model_hp, so optimise
            if self.model_hp is None:
                self.model_hp = model.param_array
            else:
                # print(self.model_hp)
                # print(model.param_array)
                # previous iter learned mix, so remove mix before setting
                if len(model.param_array) < len(self.model_hp):
                    model.param_array = self.model_hp[1:]
                else:
                    model.param_array = self.model_hp

            return False, model
