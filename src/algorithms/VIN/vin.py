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

from dataset.dataset import *
from utility.utils import *
from model import *

from domains.gridworld import *
from generators.obstacle_gen import *
import time
from astar.astar2 import main as astar
from domains.cached_maps import Maps

###############################################
def main(grid):  # or this? 

    n_domains=1
    n_traj=1
    max_obs=30,    #max number of obstacles
    max_obs_size=None, 
    n_actions=16    

    # Parsing training parameters
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--weights',
        type=str,
        default= 'trained/150000_tested_pb_vin_16x16.pth', #'trained/90000_new_pb_vin_8x8.pth',
        help='Path to trained weights')
    parser.add_argument('--plot', action='store_true', default=True) 

    #change the image size
    parser.add_argument('--imsize', type=int, default=16, help='Size of image')
    parser.add_argument(
        '--k', type=int, default=10, help='Number of Value Iterations')
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
    # parser.add_argument(
    #     '--logname',
    #     type = str,
    #     default = 'deflog.log',
    #     help = 'Logfile Name'
    # )
    config = parser.parse_args()
    print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@config  ",config)

    agent: Agent = grid.agent
    goal: Goal = grid.goal

    goal1 = [goal.position.x,goal.position.y]
    agent1 = [agent.position.x,agent.position.y]
    obstacles: Obstacle =grid.obstacles

    
    xw = grid.size.width
    yw = grid.size.height

    # make empty map
    mp = [[1 for i in range(yw)] for i in range(xw)]

    
    #make the start,goal cell 
    mp[goal.position.y][goal.position.x] = 3
    mp[agent.position.y][agent.position.x] = 2

    for ob in obstacles:
        mp[ob.position.y][ob.position.x]= 0
    
    print('***********mp   ',mp)
    mp_conv = np.asarray(mp)    
    print('$$$$$$$$$$$$$$$$ mp_conv   ',mp_conv)


    mp_conv = np.transpose(mp_conv)
    print('&&&&&&&&&&&&&&&&&&&&&& mp_conv   ',mp_conv)
    
    # Correct vs total:
    correct, total, notcorrect, counter, total_dev_rel,total_dev_non_rel, skipped = 0.0, 0.0, 0.0, 0,0.0, 0.0,0
    map_skip = []
    time_elapsed = []
    states_xy_less_than_zero = 0
    # Automatic swith of GPU mode if available
    use_GPU = torch.cuda.is_available()
    # Instantiate a VIN model
    vin = VIN(config)
    # Load model parameters
    vin.load_state_dict(torch.load(config.weights))
    # Use GPU if available
    if use_GPU:
        vin = vin.cuda()
        logging.debug('GPU Enabled')
    t0 = time.time()
    for dom in range(n_domains): #for 1 map in 100 maps
        # Randomly select goal position (x and y)
        
        #path = "./resources/test_maps/8x8/mix/" 
        #path = "./resources/test_maps/8x8/house/" 
        # path = "./resources/maps/8x8_map/" 

        # with open(path + str(dom) + '.json') as json_file:
        #     mp = json.load(json_file)
        
        # print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%mp", mp)

        # logging.debug('\n')
        # logging.debug('----------------------------------------------------')
        # logging.debug(" Map Succesffuly Loaded: " + '#: ' + str(dom))
        # goal = mp['goal']
        # agent = mp['agent']
        # logging.debug('Goal: ' + str(goal) + ' Agent: ' + str(agent))

        # mp = np.array(mp['grid'])

        # print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%mp=np.array(mp['grid'])  ", mp)

        # mp_conv = convert_map(mp_raw = mp, border = True)
        # mp_conv[goal[0],goal[1]] = 3
        # mp_conv[agent[0],agent[1]] = 2
        # mp_conv1 = np.ndarray.tolist(mp_conv)
    

        # logging.debug('Converted map: ' + str(mp_conv))

        G = gridworld(mp_conv, goal1[0], goal1[1])
        # Get value prior
        value_prior = G.get_reward_prior() #this is correct. 
        # Sample random trajectories to our goal
        #states_xy gives path of A*
        states_xy, states_one_hot = sample_trajectory(G, goal1, n_traj,agent1,mp_conv) #the error occurs here

        #print ('************************states_xy  ', states_xy)
        #print ('************************states_one_hot  ', states_one_hot)       


        logging.debug('States Returned from Sample Trajectory: ' + str(states_xy))
        #Doesn't get here! 

        for i in range(n_traj):
            #if len(states_xy) > 0:
            L= int((xw + yw) *3)
            #print("1111111111111111 state_xy",len(states_xy))
            # Get number of steps to goal
            #L = len(states_xy) * 10
            #print('STATES XY I ' + str(states_xy[i]))
            # Allocate space for predicted steps
            pred_traj = np.zeros((L, 2))
            # Set starting position
            pred_traj[0, :] =  [agent.position.x,agent.position.y] #states_xy[0, :] #starting position comes from pred trajectory [agent.position.x,agent.position.y]
            print("pred_traj[0,:1]>>>>>>>>>>>>>>>>>>>>>>>>>>>>  ", pred_traj[0,:])
            print('***************pred_traj  ',pred_traj)
            #time_1 = timer()
            for j in range(1, L):
                # Transform current state data
                state_data = pred_traj[j - 1, :]
                print('***************state_data  ',state_data)
                state_data = state_data.astype(np.int)
                # Transform domain to Networks expected input shape
                im_data = G.image.astype(np.int)
                im_data = 1 - im_data
                print('***************im_data  ',im_data)
                # print(
                #     'Im Data', im_data
                # )
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
                #This is where state_map_col and state_map_row are assigned! Its using G!!
                s = G.map_ind_to_state(pred_traj[j - 1, 0], pred_traj[j - 1, 1]) #PROBLEM
                ns = G.sample_next_state(s, a)
                nr, nc = G.get_coords(ns)
                print ("((((((((((((((((((s", s,"\n",'ns',ns,'nr',nr,"nc",nc )
                pred_traj[j, 0] = nr #nr is x coord of goal
                pred_traj[j, 1] = nc #nc is y coord of goal
                if nr == goal1[0] and nc == goal1[1]:
                    # We hit goal so fill remaining steps
                    pred_traj[j + 1:, 0] = nr
                    pred_traj[j + 1:, 1] = nc
                    return pred_traj
                    #break                   
        return pred_traj


            # time_2 = timer()
            # time_elapsed.append(time_2 - time_1)
            # Plot optimal and predicted path (also start, end) How many times you hit target
            # if pred_traj[-1, 0] == goal1[0] and pred_traj[-1, 1] == goal1[1]:
            #     correct += 1
            # total += 1
            # logging.debug('------------------')
            # logging.debug('VIN Calculated Path: ' + str(pred_traj))
            # logging.debug('A* Calculated Path: ' + str(states_xy))

            # if config.plot == True:
            #     visualize(G.image.T, states_xy, pred_traj)
            #     t_visualize = time.time()
            # dev = 0
            # dev_rel,dev_non_rel = deviation(states_xy, pred_traj,goal,counter) 
            # logging.debug('Deviation for this map: : ' + str(dev))
            # total_dev_non_rel += dev_non_rel
            # total_dev_rel += dev_rel
            # counter += 1
        # else:
        #     states_xy_less_than_zero += 1
        #     skipped += 1
        #     map_skip.append(dom)



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

        print("trace   ", trace)

        trace = np.ndarray.tolist(trace)

        print("trace", trace)

        for point in trace:
            self.move_agent(Point(int(point[0]),int(point[1])))
            self.key_frame(ignore_key_frame_skip=True)





