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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """


    # General setup for DFS
    # stack and visited are used for the actual traversal
    # path is a map of nodes -> the direction used to travel to that node
    # it is used to retrace the path we used to find the goal
    node = problem.getStartState()
    stack = util.Stack()
    stack.push((node, 0))
    visited = set()
    visited.add(node)
    path = {}

    # Fairly standard DFS
    # Break out early if we hit the goal state since we don't care about anything else
    while(not stack.isEmpty()):
        prev = node
        node = stack.pop()
        if(problem.isGoalState(node[0])):
            path[node[0]] = prev
            path[0] = node
            break
        if(node[0] not in visited):
            visited.add(node[0])
        for neighbor in problem.getSuccessors(node[0]):
            if(neighbor[0] not in visited):
                path[neighbor[0]] = node
                stack.push(neighbor)
    node = 0
    output = []
    while(node != problem.getStartState()):
        output.insert(0,path[node][1])
        node = path[node][0]
    print len(output)
    return output[1:]

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # General setup for DFS
    # stack and visited are used for the actual traversal
    # path is a map of nodes -> the direction used to travel to that node
    # it is used to retrace the path we used to find the goal
    node = problem.getStartState()
    queue = util.Queue()
    queue.push(((node,0), [node]))
    visited = set()

    # Fairly standard BFS
    # Need to pass along the path so that we can tell what path led to a given node when we get to the end
    while(not queue.isEmpty()):
        node, path = queue.pop()
        visited.add(node[0])

        if(problem.isGoalState(node)):
            break
        for neighbor in problem.getSuccessors(node[0]):
            if(neighbor[0] not in visited):
                visited.add(node[0])
                queue.push((neighbor, path + [neighbor]))
    return [x[1] for x in path[1:]]

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    start = problem.getStartState()

    queue = util.PriorityQueue()

    queue.push((0, start, []), 0)
    visited = set()

    while(not queue.isEmpty()):
        cost, curr, path = queue.pop()
        visited.add(curr)
        if(problem.isGoalState(curr)):
            return path
        for neighbor in problem.getSuccessors(curr):
            if(neighbor[0] not in visited):
                new_cost = cost + 0
                queue.push((new_cost, neighbor[0], path + [neighbor[1]]), problem.getCostOfActions(path))

    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
