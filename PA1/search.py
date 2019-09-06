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
    print [s, s, w, s, w, w, s, w]
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):

    """
        Call search function with decreasing cost so as to make priority queue LIFO
        Counter class serves to give me an object I can decrease and return at the same time
    """

    class Counter:
        def __init__(self):
            self.count = 0
        def dec(self):
            self.count -= 1
            return self.count

    counter = Counter()
    costfn = lambda path, state, problem: counter.dec()
    return search(problem, costfn)

def breadthFirstSearch(problem):
    """
        Call search function with constant cost so as to make priority queue FIFO
    """
    costfn = lambda path, state, problem: 0
    return search(problem, costfn)

def uniformCostSearch(problem):
    """
        Call search function with and forward along cost function to the problem
    """
    costfn = lambda path, state, problem: problem.getCostOfActions(path)
    return search(problem, costfn)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """
        Call search function with cost that is equal to the sum of the problems cost function and the provided heuristic
    """
    costfn = lambda path, state, problem: problem.getCostOfActions(path) + heuristic(state, problem)
    return search(problem, costfn)


def search(problem, costfn):
    start = problem.getStartState()
    queue = util.PriorityQueue()
    queue.push((start, []), 0)

    visited = set()

    while (not queue.isEmpty()):
        node, path = queue.pop()

        if (problem.isGoalState(node)):
            return path
        if node not in visited:
            for neighbor in problem.getSuccessors(node):
                state, direction, cost = neighbor
                if (state not in visited):
                    new_path = path + [direction]
                    cost = costfn(new_path, state, problem)
                    queue.push((state, new_path), cost)
            visited.add(node)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
