#!/usr/bin/env python
  
 
import sys
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from math import sqrt
import argparse

from typing import List, Tuple

import numpy as np

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.gradient_map_display import GradientMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from simulator.views.map_displays.numbers_map_display import NumbersMapDisplay
from structures import Point
  
  
class OMPL_RRT_Test(Algorithm):  
  
    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)

    
    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        return super().set_display_info()
  
    
    # Default path param is the trace produced with plan function below 
    def _find_path_internal(self, path= trace) -> None:
        """
        Read super description
        The internal implementation of :ref:`find_path`
        """


# OMPL functions
def isStateValid(state, boundary, obstacles):
    # Some arbitrary condition on the state (note that thanks to
    # dynamic type checking we can just call getX() and do not need
    # to convert state to an SE2State.)
    return state.getX() < boundary


def plan() -> List[Point]:
    """
    Read super description
    The internal implementation of :ref:`find_path`
    """
    
    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], R2
    space = ob.RealVectorStateSpace(2)

    #get the PathBench map
    grid: Map = self._get_grid()

    #agent and goal are represented by a point(x,y) and radius
    agent: Agent = grid.agent
    goal: Goal = grid.goal

    # Set the bounds of space to be in [0,1].
    space.setBounds(0.0, 1.0)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    # Set the obstacles of the map for the statevalidity function
    ##

    # Set boundary for statevalidity function
    ##

    # Set the object used to check which states in the space are valid
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    # Set robot's starting state to agent's position (x,y) -> e.g. (0,0)
    start = ob.State(space)
    #start[0] = 0.0
    #start[1] = 0.0
    start[0] = agent.position.x
    start[1] = agent.position.y

    # Set robot's goal state (x,y) -> e.g. (1.0,0.0)
    goal = ob.State(space)
    #goal[0] = 1.0
    #goal[1] = 0.0
    goal[0] = goal.position.x
    goal[1] = goal.position.y

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
    
    # ******create a planner for the defined space
    planner = og.RRTConnect(si)
    
    # set the problem we are trying to solve for the planner
    planner.setProblemDefinition(pdef)
    
    # perform setup steps for the planner
    planner.setup()
    
    # print the settings for this space
    print(si.settings())
    
    # print the problem settings
    print(pdef)
    
    # attempt to solve the problem within ten second of planning time
    solved = planner.solve(10.0)

    #metrics generation is possible here
    #

    # For troubleshooting 
    if solved:
        # get the goal representation from the problem definition (not the same as the goal state)
        # and inquire about the found path
        path = pdef.getSolutionPath()
        print("Found solution:\n%s" % path)
    else:
        print("No solution found")
  
    #return trace for _find_path_internal method
    #trace = #path -> list of points 
    #return trace


#run plan function to get path 
#trace = plan()