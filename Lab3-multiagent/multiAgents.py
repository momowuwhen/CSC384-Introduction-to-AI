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
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
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
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]
        
        "*** YOUR CODE HERE ***"
        gameBoard = successorGameState.getWalls()
        if successorGameState.isWin():
            return float("inf")
        if successorGameState.isLose():
            return float("-inf")
        
        score = 0

        ghost_dist_list = []
        for newGhostState in newGhostStates:
            newGhostPos = newGhostState.getPosition()
            ghost_dist_list.append(manhattanDistance(newGhostPos, newPos))      
        if ghost_dist_list:
            ghost_dist = min(ghost_dist_list)
            
        if all(i == 0 for i in newScaredTimes):
            if ghost_dist < 1:
                score = float("-inf")
                
        for newGhostState in newGhostStates:
            newGhostPos = newGhostState.getPosition()
            if manhattanDistance(newGhostPos,newPos) <= 1:
                score = score - 50
        
        
        curr_food = currentGameState.getFood()
        if (curr_food[newPos[0]][newPos[1]] == True):
            score = score + 25

        
        food_dist_list = []
        for i in range(gameBoard.width):
            for j in range(gameBoard.height):
                if curr_food[i][j] == True:
                    food_dist_list.append(manhattanDistance((i,j),newPos))      
        if food_dist_list:
            score = score - min(food_dist_list)


        return score

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

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"
        #util.raiseNotDefined()
        def DFMiniMax(gameState, depth, agentIndex):
            best_move = None
            if gameState.isWin() or gameState.isLose() or (depth == self.depth and agentIndex == 0):
                return best_move, self.evaluationFunction(gameState)

            if agentIndex == 0:
                value = float("-inf")
            else:
                value = float("inf")

            if agentIndex == gameState.getNumAgents() - 1:
                nxt_depth = depth + 1
            else:
                nxt_depth = depth
            
            for move in gameState.getLegalActions(agentIndex):
                nxt_gameState = gameState.generateSuccessor(agentIndex, move)
                nxt_agent = (agentIndex + 1) % gameState.getNumAgents()
                nxt_move, nxt_val = DFMiniMax(nxt_gameState, nxt_depth, nxt_agent)
                if agentIndex == 0 and value < nxt_val:
                    value, best_move = nxt_val, move
                if agentIndex > 0 and value > nxt_val:
                    value, best_move = nxt_val, move
            return best_move, value

        action = DFMiniMax(gameState, 0, 0) [0]
        return action

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        #util.raiseNotDefined()
        def AlphaBeta(gameState, depth, agentIndex, alpha, beta):
            
            best_move = None
            if gameState.isWin() or gameState.isLose() or (depth == self.depth and agentIndex == 0):
                return best_move, self.evaluationFunction(gameState)

            if agentIndex == 0:
                value = float("-inf")
            else:
                value = float("inf")

            if agentIndex == gameState.getNumAgents() - 1:
                nxt_depth = depth + 1
            else:
                nxt_depth = depth

            for move in gameState.getLegalActions(agentIndex):
                nxt_gameState = gameState.generateSuccessor(agentIndex, move)
                nxt_agentIndex = (agentIndex + 1) % gameState.getNumAgents()
                nxt_move, nxt_val = AlphaBeta(nxt_gameState, nxt_depth, nxt_agentIndex, alpha, beta)
                
                if agentIndex == 0:
                    if value < nxt_val: value, best_move = nxt_val, move
                    if value >= beta: return best_move, value
                    alpha = max(alpha, value)
                else:
                    if value > nxt_val: value, best_move = nxt_val, move
                    if value <= alpha: return best_move, value
                    beta = min(beta, value)
            
            return best_move, value

        action = AlphaBeta(gameState, 0, 0, float("-inf"), float("inf"))[0]
        return action


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
        "*** YOUR CODE HERE ***"
        #util.raiseNotDefined()
        def Expectimax(gameState, depth, agentIndex):
            best_move = None
            if gameState.isWin() or gameState.isLose() or (depth == self.depth and agentIndex == 0):
                return best_move, self.evaluationFunction(gameState)

            value = float("-inf") if agentIndex == 0 else 0
            if agentIndex == 0:
                value = float("-inf")
            else:
                value = 0

            if agentIndex == gameState.getNumAgents() - 1:
                nxt_depth = depth + 1
            else:
                nxt_depth = depth

            for move in gameState.getLegalActions(agentIndex):
                nxt_gameState = gameState.generateSuccessor(agentIndex, move)
                nxt_agentIndex = (agentIndex + 1) % gameState.getNumAgents()
                nxt_move, nxt_val = Expectimax(nxt_gameState, nxt_depth, nxt_agentIndex)

                if agentIndex == 0 and value < nxt_val:
                    value, best_move = nxt_val, move
                if agentIndex != 0:
                    prob = 1.0 / len(gameState.getLegalActions(agentIndex))
                    value = value + prob * nxt_val
            return best_move, value
        
        action = Expectimax(gameState, 0, 0)[0]
        return action


def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    #util.raiseNotDefined()
    if currentGameState.isWin():
      return float('inf')
    if currentGameState.isLose():
      return -float('inf')

    score = currentGameState.getScore()
    currPos = currentGameState.getPacmanPosition()
    
    foodPos_list = currentGameState.getFood().asList()
    food_dist = []
    for foodPos in foodPos_list:
        food_dist.append(1.0/manhattanDistance(foodPos, currPos))
    if food_dist:
        score = score + 13 * sum(food_dist) / len(food_dist)
    else:
        score = float("inf")

    ghostPos_list = []
    scaredTimes = []
    for ghost in currentGameState.getGhostStates():
        ghostPos_list.append(ghost.getPosition())
        scaredTimes.append(ghost.scaredTimer)
    ghost_dist = []
    for ghostPos in ghostPos_list:
        ghost_dist.append(manhattanDistance(ghostPos, currPos))
    if ghost_dist:
        if sum(scaredTimes) > 0:
            score = score + 0.1 * min(ghost_dist)
        else:
            score = score - 0.1 * min(ghost_dist) 

    capPos_list = currentGameState.getCapsules()
    cap_dist = []
    for capPos in capPos_list:
        cap_dist.append(1.0/manhattanDistance(capPos, currPos))
    if cap_dist:
        if sum(scaredTimes) == 0:
            score = score + 10 * sum(cap_dist) / len(cap_dist)
        else:
            score = score - 10 * sum(cap_dist) / len(cap_dist)
    
    return score 


better = betterEvaluationFunction
