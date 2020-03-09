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
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueueWithFunction
    # Data structure(Priority queue) to store the states with all the states in path to start state format: [(state,
    # action taken, cost)]
    # getCostOfActions will return count of no of direction sent to it in a list which will be
    # used as cost
    statesWithPath = PriorityQueueWithFunction(lambda states: problem.getCostOfActions([state[1] for state in states][1:0]))
    # Adding with root node
    statesWithPath.push([(problem.getStartState(), "Root", 0)])
    closedStates = set([problem.getStartState()])

    while not statesWithPath.isEmpty():
        currentStateWithPath = statesWithPath.pop()
        # Current state will be at the end that's why -1 is used
        if problem.isGoalState(currentStateWithPath[-1][0]):
            # Ignore the root while returning the directions
            return [state[1] for state in currentStateWithPath][1:]
        for suc in problem.getSuccessors(currentStateWithPath[-1][0]):
            if suc[0] not in closedStates:
                successorState = currentStateWithPath[:]
                successorState.append(suc)
                statesWithPath.push(successorState)
        closedStates.add(currentStateWithPath[-1][0])

    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
#python3 pacman.py -l tinyMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # f = g+h (final cost)
    # g = cost to reach there from starting node
    # h = guessed cost to reach the goal node from the present node (heuristic)
    openList = []
    closedList = []
    pathList = []
    startNode = [problem.getStartState(), 0, 0, 0]  # [statePosition, f, g, h]
    openList.append(startNode)
    pathList.append([startNode[0]," ",0])

    while openList:
        #Get Current Node
        presentNode = min(openList, key = lambda t: t[1])
        openList.remove(presentNode)
        closedList.append(presentNode)
        tempPath = [item[1] for item in pathList if item[0]==presentNode[0] and item[2] == presentNode[1]]
        tempPath = tempPath[0]

        #Is it a Goal Node
        if problem.isGoalState(presentNode[0]):
            return tempPath.strip().split(" ")

        #otherwise
        children = problem.getSuccessors(presentNode[0])

        for child in children:
            if child[0] in [s for s, f, g, h in closedList]:
                continue
            # Generate children and their specifications
            tempG = presentNode[2] + child[2]
            tempH = heuristic(child[0], problem)
            tempF = tempG + tempH
            tempNode = [child[0], tempF, tempG, tempH]
            tempNewPath = tempPath + " " + str(child[1])

            # If Child is already in openList
            if child[0] in [item[0] for item in openList]:
                temp = [item for item in openList if item[0] == child[0]]
                temp = temp[0]
                if temp[2] < tempG:
                    continue
            # Add a new child to the openlist
            openList.append(tempNode)
            pathList.append([child[0], tempNewPath, tempF])
    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
