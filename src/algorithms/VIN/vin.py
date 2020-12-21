from typing import List, Tuple

import numpy as np

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.configuration.entities.obstacle import Obstacle
from simulator.services.services import Services
from simulator.views.map_displays.gradient_map_display import GradientMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from simulator.views.map_displays.numbers_map_display import NumbersMapDisplay
from structures import Point
import copy



##################
import sys
import argparse
import math
import matplotlib.pyplot as plt
import pickle
import os
import json 
import numpy as np
import logging
#from timeit import default_timer as timer
import torch
from torch.autograd import Variable

from dataset import *
from utils import *
from model import *

from domains.gridworld import *
from generator.obstacle_gen import *
import time

#VIN might swap x and y

###############################################
def main(grid): 

    n_domains=1
    n_traj=1
    max_obs=30,    #max number of obstacles
    max_obs_size=None, 
    n_actions=8
    gen = False    

    if grid.size.width == 100 or grid.size.height == 100:
        k = 48
        im_size = 100
        training_file = 'trained/30k_no_block_dataset_vin_64x64.pth'
    elif grid.size.width == 64 or grid.size.height == 64:
        k = 48
        im_size = 64
        training_file = 'trained/30k_no_block_dataset_vin_64x64.pth'
    elif grid.size.width == 28 or grid.size.height == 28:
        k = 36
        im_size = 28
        training_file = 'trained/60k_no_block_att3_vin_28x28.pth' #max size for accuracy for VIN
    elif grid.size.width == 16 or grid.size.height == 16:
        k = 20 #iterations
        im_size = 16
        training_file = 'trained/60k_no_block_att3_vin_16x16.pth'
    elif grid.size.width == 8 or grid.size.height == 8:
        k = 10
        im_size = 8
        training_file = 'trained/60k_no_block_att3_vin_8x8.pth'
    else:
        k = 48
        im_size = grid.size.width
        training_file = 'trained/30k_no_block_dataset_vin_64x64.pth'
    # Parsing training parameters

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--weights',
        type=str,
        # default='trained/60k_no_block_att3_vin_8x8.pth',
        default = training_file,
        help='Path to trained weights')
    parser.add_argument('--plot', action='store_true', default=False)
    parser.add_argument('--gen', action='store_true', default=False)
    parser.add_argument('--imsize', type=int, default=im_size, help='Size of image')
    parser.add_argument(
        '--k', type=int, default=k, help='Number of Value Iterations')
    parser.add_argument(
        '--l_i', type=int, default=2, help='Number of channels in input layer')
    parser.add_argument(
        '--l_h',
        type=int,
        default=150,
        help='Number of channels in first hidden layer')
    parser.add_argument(
        '--l_q',
        type=int,
        default=10,
        help='Number of channels in q layer (~actions) in VI-module')
    config = parser.parse_args()
    # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@config  ",config)

    use_GPU = torch.cuda.is_available()
    # Instantiate a VIN model
    vin = VIN(config)
    # Load model parameters
    vin.load_state_dict(torch.load(config.weights))
    # Use GPU if available
    if use_GPU:
        vin = vin.cuda()
    counter,total_no_soln = 0,0
    global data
    data = []
    t_list = []
    total_dev_non_rel, total_dev_rel = 0.0,0.0
    total_dist, total_astar_dist = 0.0,0.0
    metrics = True #this enables displaying the distance left to reach goal upon a failure 
    dist_remain_avg = 0.0
    for dom in range(n_domains):
        #if gen: #internal generator
        #    goal = [
        #    np.random.randint(config.imsize),
        #    np.random.randint(config.imsize)
        #    ]   
        #    obs = obstacles([config.imsize, config.imsize], goal, max_obs_size)
        #    # Add obstacles to map
        #    n_obs = obs.add_n_rand_obs(max_obs)
        #    # Add border to map
        #    border_res = obs.add_border()
        #    # Ensure we have valid map
        #    if n_obs == 0 or not border_res:
        #        continue
        #    start = None
        #else:
        # wpn = True Flag for metric
        # path = './resources/maps/'
        # path = './resources/testing_maps/8x8/'
        # mp, goal, start = open_map(dom,path)
        print(grid)
        mp = grid.grid
        goal = [grid.goal.position.x,grid.goal.position.y]
        start = [grid.agent.position.x,grid.agent.position.y]
        # path = './maps/8_data_300'
        # mp, goal, start = open_map_list(dom,path)
        mp[start[1]][start[0]] = 0 #Set the start position as freespace too
        mp[goal[1]][goal[0]] = 0 #Set the goal position as freespace too

        goal = [goal[1],goal[0]] #swap them around, for the row col format (x = col not row)
        start = [start[1],start[0]]
        obs = obstacles([config.imsize, config.imsize], goal, max_obs_size)
        obs.dom = mp
            # print('Goal:', goal,'agent', start, 'gridb4im', mp)
        # Get final map
        #Between mp and im, the 0 should become 1., and the 1 should become 0.
        im = obs.get_final()

        # print('Grid: ', mp, '\n Agent', start, '\n Goal', goal)
        # print('Im:', im)
        #1 is obstacles. 
        #set obs.dom as the mp

        # Generate gridworld from obstacle map
        # print(' im: %s ', im)
        # print('goal:', goal)
        # print('goal:', start)
        G = gridworld(im, goal[0], goal[1]) #Map that VIN uses, mem intensive
        # Get value prior
        # print('144')
        value_prior = G.get_reward_prior()
        # Sample random trajectories to our goal

        states_xy, states_one_hot = sample_trajectory(G, n_traj,start,gen) #dijkstra trajectory 
        # print('states_xy', states_xy[0] , len(states_xy[0]))
        if gen and len(states_xy[0]) > 0:
            save_image(G.image,(goal[0],goal[1]),states_xy[0][0],states_xy, states_one_hot,counter) #this saves the maps 
        
        counter += 1

        #algorithm part
        for i in range(n_traj):
            if len(states_xy[i]) > 1:
                t0 = time.time()
                # Get number of steps to goal
                L = len(states_xy[i]) * 2
                # Allocate space for predicted steps
                pred_traj = np.zeros((L, 2))
                # Set starting position
                pred_traj[0, :] = states_xy[i][0, :]

                for j in range(1, L):
                    # Transform current state data
                    state_data = pred_traj[j - 1, :]
                    state_data = state_data.astype(np.int)
                    # Transform domain to Networks expected input shape
                    im_data = G.image.astype(np.int)
                    im_data = 1 - im_data
                    im_data = im_data.reshape(1, 1, config.imsize,
                                              config.imsize)
                    # Transfrom value prior to Networks expected input shape
                    value_data = value_prior.astype(np.int)
                    value_data = value_data.reshape(1, 1, config.imsize,
                                                    config.imsize)
                    # Get inputs as expected by network
                    X_in = torch.from_numpy(
                        np.append(im_data, value_data, axis=1)).float()
                    S1_in = torch.from_numpy(state_data[0].reshape(
                        [1, 1])).float()
                    S2_in = torch.from_numpy(state_data[1].reshape(
                        [1, 1])).float()
                    # Send Tensors to GPU if available
                    if use_GPU:
                        X_in = X_in.cuda()
                        S1_in = S1_in.cuda()
                        S2_in = S2_in.cuda()
                    # Wrap to autograd.Variable
                    X_in, S1_in, S2_in = Variable(X_in), Variable(
                        S1_in), Variable(S2_in)
                    # Forward pass in our neural net
                    _, predictions = vin(X_in, S1_in, S2_in, config)
                    _, indices = torch.max(predictions.cpu(), 1, keepdim=True)
                    a = indices.data.numpy()[0][0]
                    # Transform prediction to indices
                    s = G.map_ind_to_state(pred_traj[j - 1, 0],
                                           pred_traj[j - 1, 1])
                    ns = G.sample_next_state(s, a)
                    nr, nc = G.get_coords(ns)
                    pred_traj[j, 0] = nr
                    pred_traj[j, 1] = nc
                    if nr == goal[0] and nc == goal[1]:
                        # We hit goal so fill remaining steps
                        pred_traj[j + 1:, 0] = nr
                        pred_traj[j + 1:, 1] = nc
                        break
                # Plot optimal and predicted path (also start, end)
                if pred_traj[-1, 0] == goal[0] and pred_traj[-1, 1] == goal[1]:
                    print('success!')
                    # print('pred_traj', pred_traj)
                return pred_traj
    return pred_traj

                # Plot optimal and predicted path (also start, end)
        #         if pred_traj[-1, 0] == goal[0] and pred_traj[-1, 1] == goal[1]:
        #             # correct += 1
        #             t1 = time.time()
        #             t_list.append(t1-t0)
        #             dev_rel,dev_non_rel,dist,astar_dist = deviation(states_xy[i],pred_traj,goal,total)
        #             total_dev_rel += dev_rel
        #             total_dev_non_rel += dev_non_rel
        #             total_dist += dist
        #             total_astar_dist += astar_dist
        #             if config.plot == True:
        #                 visualize(G.image.T, states_xy[i], pred_traj)
        #         elif metrics:
        #             d = dist_left(pred_traj,goal)
        #             dist_remain_avg += d
        #             if config.plot == True:
        #                 visualize(G.image.T, states_xy[i], pred_traj)
        #         total += 1



        #     elif wpn:
        #         total_no_soln += 1
        # sys.stdout.write("\r" + str(int(
        #     (float(dom) / n_domains) * 100.0)) + "%")
        # sys.stdout.flush()

    # sys.stdout.write("\n")
    # if total and correct:
    #     logging.info('Rollout Accuracy: %s',(100 * (correct / total)))
    #     logging.info('Rollout Accuracy Adjusted: %s',(100 * (correct / (total+total_no_soln))))
    #     logging.info('Total maps with no soln from Dijkstra %s', total_no_soln)
    #     logging.info('Total avg Rel Deviation %s', (total_dev_rel/total))
    #     logging.info('Total avg Non-Rel Deviation %s', (total_dev_non_rel/total))
    #     logging.info('Total avg VIN Distance %s', (total_dist/total))
    #     logging.info('Total avg Dijkstra Distance %s', (total_astar_dist/total))
    #     logging.info('Avg deviation from Dijkstra: %s', ((((total_astar_dist/total))-((total_dist/total)))/((total_astar_dist/total))))
    #     logging.info('Total elapsed time %s', (sum(t_list)/(correct)))
    #     logging.info('Avg distance left when failed: %s ', (dist_remain_avg/(total-correct)) )
    #     logging.info('---------------------------------Done ------------------------------------')

    # else:
    #     logging.info('No successes either vin or dijkstra')


class VINTest(Algorithm):
    step_grid: List[List[int]]


    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)
        self.step_grid = []

    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        display_info: List[MapDisplay] = super().set_display_info() #+ [
        #     GradientMapDisplay(self._services, self.step_grid,
        #                                        min_color=Wavefront.STEP_GRID_MIN_COLOR,
        #                                        max_color=Wavefront.STEP_GRID_MAX_COLOR),
        
            #NumbersMapDisplay(self._services, copy.deepcopy(self.step_grid))
        #]
        return display_info   


    # noinspection PyUnusedLocal
    def _find_path_internal(self) -> None:
        """
        Read super description
        The internal implementation of :ref:`find_path`
        """
        #._get_grid() is in Algorithm class and gets the map
        grid: Map = self._get_grid()

        trace = main(grid)

        # print("trace   ", trace)

        trace = np.ndarray.tolist(trace)

        # print("trace", trace)

        for point in trace:
            self.move_agent(Point(int(point[1]),int(point[0]))) #swap them around
            self.key_frame(ignore_key_frame_skip=True)




