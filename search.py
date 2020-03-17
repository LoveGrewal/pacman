# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def searchTree(problem, data_structure):

    statesWithPath = data_structure
    # Adding with root node
    statesWithPath.push([(problem.getStartState(), "Root", 0)])
    closedStates = set([problem.getStartState()])
    cloesedAndInProgressStates = set([problem.getStartState()])

    while not statesWithPath.isEmpty():
        currentStateWithPath = statesWithPath.pop()
        # Current state will be at the end that's why -1 is used
        if problem.isGoalState(currentStateWithPath[-1][0]):
            # Ignore the root while returning the directions
            return [state[1] for state in currentStateWithPath][1:]
        for suc in problem.getSuccessors(currentStateWithPath[-1][0]):
            if suc[0] not in cloesedAndInProgressStates:
                if not problem.isGoalState(suc[0]):
                    cloesedAndInProgressStates.add(suc[0])
                successorState = currentStateWithPath[:]
                successorState.append(suc)
                statesWithPath.push(successorState)
        closedStates.add(currentStateWithPath[-1][0])

    return []

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    from util import Stack
    stack = Stack()
    visited = []
    path = Stack()
    startState = problem.getStartState()

    #if starting state is a goal state
    if problem.isGoalState(startState):
        return []

    #Otherwise find the path for the goal state
    stack.push(startState)
    visited.append(startState)
    path.push(' ')
    while not stack.isEmpty():
        tempPresentState = stack.pop()
        tempPath = path.pop()
        visited.append(tempPresentState)
        # print("\nPopped node::")
        # print(tempPresentState)
        # print("\nPopped path::")
        # print(tempPath)
        # print("\n\n")
        if problem.isGoalState(tempPresentState):
            return tempPath.split()

        for state in problem.getSuccessors(tempPresentState):
            if state[0] not in visited:
                stack.push(state[0])
                path.push(tempPath + " " + state[1])
    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    search_technique = Queue()
    return searchTree(problem, search_technique)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueueWithFunction
    # Data structure(Priority queue) to store the states with all the states in path to start state format: [(state,
    # action taken, cost)]
    # getCostOfActions will return count of no of direction sent to it in a list which will be
    def calculateCost(states):
        cost = 0
        #length = len(states)
        for c in [state[2] for state in states]:
            cost = cost + c
        return cost
    # used as cost
    search_technique = PriorityQueueWithFunction(lambda states: calculateCost(states))
    return searchTree(problem, search_technique)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
#python3 pacman.py -l tinyMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    from util import PriorityQueueWithFunction
    # Data structure(Priority queue) to store the states with all the states in path to start state format: [(state,
    # action taken, cost)]
    # getCostOfActions will return count of no of direction sent to it in a list which will be
    def calculateCost(states):
        cost = 0
        #length = len(states)
        for c in [state[2] for state in states]:
            cost = cost + c
        heuristic_cost = heuristic(states[-1][0], problem)
        cost += heuristic_cost
        return cost
    # used as cost
    search_technique = PriorityQueueWithFunction(lambda states: calculateCost(states))
    return searchTree(problem, search_technique)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
