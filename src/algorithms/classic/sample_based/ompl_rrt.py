#!/usr/bin/env python
  
import numpy as np


from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.configuration.configuration import Configuration
from simulator.services.services import Services
from simulator.views.map.display.gradient_list_map_display import GradientListMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.numbers_map_display import NumbersMapDisplay
from structures import Point
import torch
from algorithms.classic.sample_based.core.sample_based_algorithm import SampleBasedAlgorithm
from algorithms.classic.sample_based.core.vertex import Vertex
from algorithms.classic.sample_based.core.graph import Forest


# try:
#     from ompl import util as ou
#     from ompl import base as ob
#     from ompl import geometric as og
# except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
import sys    
from os.path import abspath, dirname, join
#sys.path.insert(1, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
sys.path.insert(1,'/home/bruce/omplapp-1.4.2-Source/ompl/py-bindings')
#print(sys.path)
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import argparse
from typing import List, Tuple

#set the time the algorithm runs for
time = 40.0


def list2vec(l):
    ret = ou.vectorDouble()
    for e in l:
        ret.append(e)
    return ret

class ProjectionEvaluator(ob.ProjectionEvaluator):
    
    def __init__(self, space):
        super().__init__(space)
        self.defaultCellSizes()

    def getDimension(self):
        return 2

    def defaultCellSizes(self):
        #self.cellSizes_ = [1.0,1.0]
        self.cellSizes_ = list2vec([1.0, 1.0])

    def project(self, state, projection):
        projection[0] = state[0]
        projection[1] = state[1]


# OMPL functions
def isStateValid(state):
    # Some arbitrary condition on the state (note that thanks to
    # dynamic type checking we can just call getX() and do not need
    # to convert state to an SE2State.)
    #if state[0] > grid.size.width  and state[0] < 0.0  and state[1] < 0.0 and state[1] > grid.size.height:
    # config = Configuration()
    # services = Services(config)   
    # cls = OMPL_KPIECE_Test(services)
    # grid = cls._get_grid()

    if state[0] > x.size.width  and state[0] < 0.0  and state[1] < 0.0 and state[1] > x.size.height:
        return False

    lst = []
    for point in x.obstacles:
        lst.append((point.position.x,point.position.y))
        
    for coord in lst:
        if state[0] == coord[0] and state[1] == coord[1]:
            return False
        if round(state[0])==coord[0] and round(state[1]) == coord[1]:
            return False
    # if state[0] <28.0 and state[0]>4.0 and state[1]<19 and state[1]>4:
    #     for ob in lst:
    #         visited.append((ob.position.x,ob.position.y))   
    return True



def plan(grid):

    #agent and goal are represented by a point(x,y) and radius
    global x
    global time
    x = grid
    agent: Agent = grid.agent
    goal: Goal = grid.goal

    # Construct the robot state space in which we're planning. R2
    space = ob.RealVectorStateSpace(2)

    # Set the bounds of space to be inside Map
    bounds = ob.RealVectorBounds(2)

    # projection
    pj= ProjectionEvaluator(space)
    print('pj=',pj)
  
    pj.setCellSizes(list2vec([1.0, 1.0]))
    space.registerDefaultProjection(pj)

    # Construct the robot state space in which we're planning.
    bounds.setLow(0,0)    #set min x to _    0
    bounds.setHigh(0,grid.size.width)  #set max x to _    x width 
    bounds.setLow(1,0)    #set min y to _    0
    bounds.setHigh(1,grid.size.height)  #set max y to _    y height

    space.setBounds(bounds)

    print("bounds=",bounds.getVolume())

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    # Set the object used to check which states in the space are valid
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    # Set robot's starting state to agent's position (x,y) -> e.g. (0,0)
    start = ob.State(space)
    start[0] = float(agent.position.x)
    start[1] = float(agent.position.y)

    print(start[0],start[1])
    # Set robot's goal state (x,y) -> e.g. (1.0,0.0)
    goal = ob.State(space)
    goal[0] = float(grid.goal.position.x)
    goal[1] = float(grid.goal.position.y)

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
    
    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    #pdef.setOptimizationObjective(allocateObjective(si, objectiveType))    

    # ******create a planner for the defined space
    planner = og.RRT(si)
    
    # set the problem we are trying to solve for the planner
    planner.setProblemDefinition(pdef)

    print(planner)
    #print('checking projection',planner.getProjectionEvaluator())

    print("__________________________________________________________")
    # perform setup steps for the planner
    planner.setup()


    # print the settings for this space
    print("space settings\n")
    print(si.settings())
    
    print("****************************************************************")
    print("problem settings\n")
    # print the problem settings
    print(pdef)

    # attempt to solve the problem within ten second of planning time
    solved = planner.solve(time)

    # For troubleshooting 
    if solved:
        # get the goal representation from the problem definition (not the same as the goal state)
        # and inquire about the found path
        path = pdef.getSolutionPath()
        print("Found solution:\n%s" % path)
         #return trace for _find_path_internal method
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        
        if path is None:
            return None
            
        x= pdef.getSolutionPath().printAsMatrix()
        lst = x.split()
        lst = [int(round(float(x),0)) for x in lst]

        print(x)
        print(lst)
        
        trace = []
        for i in range(0, len(lst), 2):
            trace.append(Point(lst[i], lst[i + 1]))
            print(trace)
        return trace
    else:
        print("No solution found")


# OMPL Algorithm Class 
class OMPL_RRT(Algorithm):
    #trace: List[Point] = plan()
  
    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)
        

    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        return super().set_display_info()


    def _find_path_internal(self) -> None:
        """
        Read super description
        The internal implementation of :ref:`find_path`
        """
        grid: Map = self._get_grid()

        for obj in grid.obstacles:
            print ('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>', obj.position.x,' ',obj.position.y)

        trace = None
        #pass map used to plan to generate a list of points as path to goal 
        while trace is None:
            trace = plan(grid) 


        for point in trace:
            
            self.move_agent(point)
            self.key_frame(ignore_key_frame_skip=True)


            if grid.is_goal_reached(point,grid.goal):
                break




