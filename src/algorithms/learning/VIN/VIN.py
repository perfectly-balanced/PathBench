import sys
import random
import numpy as np
import torch
import logging
import time
import math
from torch.autograd import Variable
from typing import Dict, List, Type

from .utility.utils import *

from .model import *
from .domains.gridworld import *
from .generators.obstacle_gen import *

from algorithms.algorithm import Algorithm
from simulator.services.services import Services
from algorithms.basic_testing import BasicTesting
from structures.point import Point

class VINConfig:
    def __init__(self, l_i=2, l_h=150, l_q=10, k=10):
        self.l_i = l_i
        self.l_h = l_h
        self.l_q = l_q
        self.k = k

class VINAlgorithm(Algorithm):
    cached_models: Dict[int, Type[VIN]]
    use_GPU: bool

    def __init__(self, services: Services, testing: BasicTesting = None, config: VINConfig = VINConfig(), load_name: str = "VIN"):
        super().__init__(services, testing)
        self.cached_models = {}
        self.use_GPU = torch.cuda.is_available()
        self._load_name = load_name
        self.config = config

    def set_display_info(self):
        return super().set_display_info() + [
            
        ]

    def _find_path_internal(self) -> None:
        mp = self._get_grid()
        assert mp.size[0] == mp.size[1] and len(mp.size) == 2, \
            f"VIN only accepts square 2D maps, map size {mp.size}"
        imsize = mp.size[0]
        grid = np.copy(mp.grid)
        self.config.imsize = imsize
        model: VIN = self.load_VIN(mp.size[0])
        start: Tuple[int] = (mp.agent.position.x, mp.agent.position.y)
        goal: Tuple[int] = (mp.goal.position.x, mp.goal.position.y)

        grid[mp.agent.position.x, mp.agent.position.y] = 0 #Set the start position as freespace too
        grid[mp.goal.position.x, mp.goal.position.y] = 0 #Set the goal position as freespace too

        obs = obstacles([imsize, imsize], goal)
        obs.dom = grid

        im = obs.get_final()
        G = gridworld(im, goal[0], goal[1])
        # =======
        value_prior = G.get_reward_prior()
        # Sample random trajectories to our goal
        states_xy, states_one_hot = sample_trajectory(G, 1, start, False) #dijkstra trajectory 
        # print('states_xy', states_xy[0] , len(states_xy[0]))
        
        i = 0
        if len(states_xy[i]) > 1:
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
                im_data = im_data.reshape(1, 1, imsize,
                                            imsize)
                # Transfrom value prior to Networks expected input shape
                value_data = value_prior.astype(np.int)
                value_data = value_data.reshape(1, 1, imsize,
                                                imsize)
                # Get inputs as expected by network
                X_in = torch.from_numpy(
                    np.append(im_data, value_data, axis=1)).float()
                S1_in = torch.from_numpy(state_data[0].reshape(
                    [1, 1])).float()
                S2_in = torch.from_numpy(state_data[1].reshape(
                    [1, 1])).float()
                # Send Tensors to GPU if available
                if self.use_GPU:
                    X_in = X_in.cuda()
                    S1_in = S1_in.cuda()
                    S2_in = S2_in.cuda()
                # Wrap to autograd.Variable
                X_in, S1_in, S2_in = Variable(X_in), Variable(
                    S1_in), Variable(S2_in)
                # Forward pass in our neural net
                _, predictions = model(X_in, S1_in, S2_in, self.config)
                _, indices = torch.max(predictions.cpu(), 1, keepdim=True)
                a = indices.data.numpy()[0][0]
                # Transform prediction to indices
                s = G.map_ind_to_state(pred_traj[j - 1, 0],
                                        pred_traj[j - 1, 1])
                ns = G.sample_next_state(s, a)
                nr, nc = G.get_coords(ns)
                pred_traj[j, 0] = nr
                pred_traj[j, 1] = nc
                self.move_agent(Point(nr, nc))
                self.key_frame(True)
                if nr == goal[0] and nc == goal[1]:
                    # We hit goal so fill remaining steps
                    pred_traj[j + 1:, 0] = nr
                    pred_traj[j + 1:, 1] = nc
                    break
            # Plot optimal and predicted path (also start, end)
            if pred_traj[-1, 0] == goal[0] and pred_traj[-1, 1] == goal[1]:
                self.move_agent(self._get_grid().goal.position)
                self.key_frame(True)
                return
            self.key_frame(True)

    def load_VIN(self, size):
        if size in self.cached_models: return self.cached_models[size]
        load_fname = f"{self._load_name}_{size}x{size}.pth"
        load_path = self._services.resources.model_dir._full_path() + load_fname
        vin = VIN(self.config)
        vin.load_state_dict(torch.load(load_path, map_location=None if self.use_GPU else torch.device("cpu")))
        if self.use_GPU: vin = vin.cuda()
        self.cached_models[size] = vin
        return vin


def visualize(dom, states_xy, pred_traj):
    fig, ax = plt.subplots()
    implot = plt.imshow(dom, cmap="Greys_r")
    ax.plot(states_xy[:, 0], states_xy[:, 1], c='b', label='Optimal Path')
    ax.plot(
        pred_traj[:, 0], pred_traj[:, 1], '-X', c='r', label='Predicted Path')
    ax.plot(states_xy[0, 0], states_xy[0, 1], '-o', label='Start')
    ax.plot(states_xy[-1, 0], states_xy[-1, 1], '-s', label='Goal')
    legend = ax.legend(loc='upper right', shadow=False)
    for label in legend.get_texts():
        label.set_fontsize('x-small')  # the legend text size
    for label in legend.get_lines():
        label.set_linewidth(0.5)  # the legend line width
    plt.draw()
    plt.waitforbuttonpress(0)
    plt.close(fig)


def save_image(im, goal, start,states_xy,states_one_hot,counter):
    '''
    Saves the data made by generator as jsons. 
    '''
    s = config.imsize

    if len(states_xy[0]) == 0:

        im.tolist()[start_x][start_y] = 1
        start_xy = [0,0]
        mp = {
        'grid': im.tolist(),
        'goal': [goal[0],goal[1]],
        # 'start': int(start),
        'agent': start_xy}
        # 'states_xy': states_xy[0].tolist(),
        # 'states_one_hot': states_one_hot[0].tolist()
    else:
        mp = {
            'grid': im.tolist(),
            'goal': [goal[0],goal[1]],
            # 'start': int(start),
            'agent': states_xy[0][0].tolist()
            # 'states_xy': states_xy[0].tolist(),
            # 'states_one_hot': states_one_hot[0].tolist()   
    }
    data.append(mp)
    with open('./maps/' +str(s) + '_data_300' +  '.json', 'w') as outfile:
        json.dump(data,outfile)

def open_map(dom,path):
    '''
    Used to open a map json given dom and path, returns grid, goal and agent
    '''
    with open(str(path) + str(dom) +'.json') as json_file:
        data = json.load(json_file)
        logging.info('Opening file: ' + str(path) + str(dom) + '.json' )
        return data['grid'], data['goal'], data['agent']

def open_map_list(dom,path):
    with open(str(path) + '.json') as json_file:
        data = json.load(json_file)
        logging.info('Opening file: ' + str(path) + str(dom) + '.json' )
        return data[dom]['grid'], data[dom]['goal'], data[dom]['agent']

def deviation(optimal_path, pred_path,goal, map_num):
    optimal_path = np.array(optimal_path)
    optimal_path = 1.0 * optimal_path

    optimal_path_x = np.array(optimal_path[:,0])
    optimal_path_y = np.array(optimal_path[:,1])

    pred_path = np.unique(pred_path, axis=0) #removes duplicates at the end (when it reaches goal)

    #print('Shortened path' , pred_path)
    pred_path_x = np.array(pred_path[:,0])
    pred_path_y = np.array(pred_path[:,1])
    dist = 0.0
    astar_dist = 0.0
    prev = pred_path[0,:]
    total_diff_gen = 0
    for xy in pred_path[:,:]:

        diff = math.sqrt( ((1.0 * xy[0]- 1.0*prev[0])**2)+((1.0*xy[1] - 1.0*prev[1])**2))
        total_diff_gen += diff
        dist+= ((xy[0]-prev[0])**2 + (xy[1]-prev[1])**2)**0.5
        prev = xy 

    #prev = [0,0]
    #print('opt', optimal_path[0,:])
    prev = optimal_path[0,:]
    total_diff_optim = 0
    for xy in optimal_path[:,:]:
        # print('xy', xy)
        diff2 = math.sqrt( ((1.0 * xy[0]- 1.0*prev[0])**2)+((1.0*xy[1] - 1.0*prev[1])**2))
        total_diff_optim += diff2
        astar_dist+= ((xy[0]-prev[0])**2 + (xy[1]-prev[1])**2)**0.5
        prev = xy 
    
    dev_non_rel = abs(total_diff_optim-total_diff_gen)
    dev_rel = dev_non_rel/total_diff_optim #TODO: Add avg distance of gen trajectory
    return(dev_rel,dev_non_rel,dist,astar_dist)

def dist_left(pred_traj, goal):
    '''
    Finds the distance left between the point and the goal
    '''
    pred_traj = np.array(pred_traj) #euclidean distance or geometric distance ? use geometric
    x1,y1 = pred_traj[-1][0], pred_traj[-1][1]
    x2,y2 = goal[0],goal[1]
    dist = (((x2-x1)**2 + (y2-y1)**2))**0.5
    return dist        


if __name__ == '__main__':
    # Parsing training parameters
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--weights',
        type=str,
        default='trained/vin_8x8.pth',
        help='Path to trained weights')
    parser.add_argument(
        '--maps',
        type=str,
        default='resources/testing_maps/16x16',
        help='Path to maps')
    parser.add_argument('--plot', action='store_true', default=False)
    parser.add_argument('--gen', action='store_true', default=False)
    parser.add_argument('--imsize', type=int, default=8, help='Size of image')
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
    config = parser.parse_args()
    # Compute Paths generated by network and plot
