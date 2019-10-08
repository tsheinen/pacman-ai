# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
      A reflex agent chooses an action at each choice point by examining
      its alternatives via a state evaluation function.

      The code below is provided as a guide.  You are welcome to change
      it in any way you see fit, so long as you don't touch our method
      headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        oldPos = currentGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        currentFood = currentGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        cost = 0;
        ghostDistances = [util.manhattanDistance(newPos, x.getPosition()) for x in newGhostStates]
        if min([x.scaredTimer for x in currentGameState.getGhostStates()]) > 0:
            cost -= min(ghostDistances) * 3
        else:
            cost -= sum([x+1 if x < 2 else 0 for x in ghostDistances]) * 3
            cost -= min([util.manhattanDistance(newPos, x) for x in currentFood.asList()])
            caps = [util.manhattanDistance(newPos, x) for x in currentGameState.getCapsules()]
            cost -= min(caps) if len(caps) > 0 else 0
        return cost

def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
      This class provides some common elements to all of your
      multi-agent searchers.  Any methods defined here will be available
      to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

      You *do not* need to make any changes here, but you can if you want to
      add functionality to all your adversarial search agents.  Please do not
      remove anything, however.

      Note: this is an abstract class: one that should not be instantiated.  It's
      only partially specified, and designed to be extended.  Agent (game.py)
      is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
      Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action from the current gameState using self.depth
          and self.evaluationFunction.

          Here are some method calls that might be useful when implementing minimax.

          gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

          gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

          gameState.getNumAgents():
            Returns the total number of agents in the game
        """
        def max_agent(state, depth):
            actions = state.getLegalActions(0)
            if (len(actions) == 0):
                return state.getScore()

            scores = [(action, min_agent(state.generateSuccessor(0, action), depth, 1, )) for action in actions]


            bestAction, bestScore = max(scores, key = lambda x: x[1])
            return bestAction if depth == 0 else bestScore

        def min_agent(state, depth, ghost):
            actions = state.getLegalActions(ghost)
            if (len(actions) == 0):
                return state.getScore()
            nextGhost = (ghost + 1) % state.getNumAgents()

            def eval(action):
                if (nextGhost == 0):
                    if (depth == self.depth - 1):
                        return self.evaluationFunction(state.generateSuccessor(ghost, action))
                    else:
                        return max_agent(state.generateSuccessor(ghost, action), depth + 1)
                else:
                    return min_agent(state.generateSuccessor(ghost, action), depth, nextGhost)
            scores = [eval(action) for action in actions]
            return min(scores)
        score = max_agent(gameState, 0)
        return score

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
      Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action using self.depth and self.evaluationFunction
        """
        def max_agent(state, depth, alpha, beta):
            actions = state.getLegalActions(0)
            if (len(actions) == 0):
                return state.getScore()

            best = (float('-inf'), Directions.STOP)
            for action in actions:
                score = min_agent(state.generateSuccessor(0, action), depth, 1, alpha, beta)
                best = max(best, (score, action), key = lambda x: x[0])
                alpha = max(alpha, best[0])
                if best[0] > beta:
                    return best[0]
            return best[1] if depth == 0 else best[0]

        def min_agent(state, depth, ghost, alpha, beta):
            actions = state.getLegalActions(ghost)
            if (len(actions) == 0):
                return state.getScore()
            nextGhost = (ghost + 1) % state.getNumAgents()

            def eval(action):
                if (nextGhost == 0):
                    if (depth == self.depth - 1):
                        return self.evaluationFunction(state.generateSuccessor(ghost, action))
                    else:
                        return max_agent(state.generateSuccessor(ghost, action), depth + 1, alpha, beta)
                else:
                    return min_agent(state.generateSuccessor(ghost, action), depth, nextGhost, alpha, beta)

            best = (float('inf'), Directions.STOP)
            for action in actions:
                score = eval(action)
                best = min(best, (score, action), key = lambda x: x[0])
                beta = min(beta, best[0])
                if best[0] < alpha:
                    return best[0]
            return best[0]
        score = max_agent(gameState, 0, float("-inf"), float("inf"))
        return score

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
          Returns the expectimax action using self.depth and self.evaluationFunction

          All ghosts should be modeled as choosing uniformly at random from their
          legal moves.
        """
        def max_agent(state, depth):
            actions = state.getLegalActions(0)
            if (len(actions) == 0):
                return state.getScore()
            scores = [(action, min_agent(state.generateSuccessor(0, action), depth, 1, )) for action in actions]
            bestAction, bestScore = max(scores, key = lambda x: x[1])
            return bestAction if depth == 0 else bestScore

        def min_agent(state, depth, ghost):
            actions = state.getLegalActions(ghost)
            if (len(actions) == 0):
                return state.getScore()
            nextGhost = (ghost + 1) % state.getNumAgents()
            def eval(action):
                if (nextGhost == 0):
                    if (depth == self.depth - 1):
                        score = self.evaluationFunction(state.generateSuccessor(ghost, action))
                        return score
                    else:
                        score = max_agent(state.generateSuccessor(ghost, action), depth + 1)
                        return score
                else:
                    score = min_agent(state.generateSuccessor(ghost, action), depth, nextGhost)
                    return score
            scores = [eval(action) for action in actions]
            return sum(scores) /float(len(scores))
        return max_agent(gameState, 0)



def betterEvaluationFunction(currentGameState):
    """
      Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
      evaluation function (question 5).

      DESCRIPTION: <write something here so we know what you did>
    """
    score = 0

    score = currentGameState.getScore()

    return score

# Abbreviation
better = betterEvaluationFunction

