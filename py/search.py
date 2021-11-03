# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
from util import heappush, heappop
class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
      """
      Returns the start state for the search problem
      """
      util.raiseNotDefined()

    def isGoalState(self, state):
      """
      state: Search state

      Returns True if and only if the state is a valid goal state
      """
      util.raiseNotDefined()

    def getSuccessors(self, state):
      """
      state: Search state

      For a given state, this should return a list of triples,
      (successor, action, stepCost), where 'successor' is a
      successor to the current state, 'action' is the action
      required to get there, and 'stepCost' is the incremental
      cost of expanding to that successor
      """
      util.raiseNotDefined()

    def getCostOfActions(self, actions):
      """
      actions: A list of actions to take

      This method returns the total cost of a particular sequence of actions.  The sequence must
      be composed of legal moves
      """
      util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches
    the goal. Make sure that you implement the graph search version of DFS,
    which avoids expanding any already visited states. 
    Otherwise your implementation may run infinitely!
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    """
    YOUR CODE HERE
    """
   

    closedtSet = []  # keeps track of the nodes that have been visited
    openSet = [problem.getStartState()] # openSt will behave as a stack. Hence, when we explore the nodes we will go to the immideiate child
    dict = {}  # map to trace back the actions thar are going to pass to the search agent
    actions = []  # the list of actions
    
    while len(openSet) > 0:

        state = openSet.pop()  # first pop the value from the frige
        if problem.isGoalState(state):  # if it is the goal state done
            print("found goal!")

            while state != problem.getStartState():  # action logic management
                actions.append(dict[state][1]) # backtrack up the map to construct the path
                state = dict[state][0]
            actions.reverse() # the path is inversed so reverse the actions list and then return
            return actions

        else:
            if state not in closedtSet:  # only append the children to the frige if the parent has not been added to the list of visited nodes
                closedtSet.append(state)   
                for (next_state, action, cost) in problem.getSuccessors(state): # for each child add it to the stack
                    openSet.append(next_state)
                    if(next_state not in closedtSet): # only add the child and the the action to the map if the parent has not been visited yet. 
                        dict[next_state] = (state, action)

    util.raiseNotDefined()


def breadthFirstSearch(problem):

    closedtSet = []  # keeps track of the nodes that have been visited
   
    openSet = [problem.getStartState()] # openset will behave like a queue hence the value that will be removed is the first value that was appended
    dict = {}  # map to trace back the actions thar are going to pass to the search agent
    actions = []  # the list of actions
    while len(openSet) > 0:
        state = openSet.pop(0)    # first pop from the start of the list. hecne
        if problem.isGoalState(state):  # if it is the goal state done
            print("Found Goal")

            while state != problem.getStartState():  # action logic management

                actions.append(dict[state][1]) # backtrack the map to construct the list of actions
                state = dict[state][0]
            actions.reverse()    # reverse the list of actions as we had backtracked

            return actions

        else:
            if state not in closedtSet:  # only append the children to the frige if the value has not been added to the list of visited nodes
                closedtSet.append(state)

                for (next_state, action, cost) in problem.getSuccessors(state):

                    openSet.append(next_state)  # add the children to the list 

                    if(next_state not in dict.keys()):  # in BFS we need to keep track of the original parent for the node. Hence the original parent should never change
                                                        #else the list of actions being created would be a lot like DFS
                        dict[next_state] = (state, action)

    util.raiseNotDefined()


def uniformCostSearch(problem):

    closedtSet = []  # keeps track of the nodes that have been visited
    openSet = []  # keeps track of childrent added to the fringe
    heappush(openSet, (0, problem.getStartState())) # openset will now store the tuples of cost and state. Hence the least cost would be given the 
                                                    # most priority when it is being removed from the fringe

    dict = {}  # map to trace back the actions thar are going to pass to the search agent
    actions = []  # the list of actions
    print(problem)
    while len(openSet) > 0:
        state = heappop(openSet)    # using heapop to remove based on the cost 
        backtrack = state[1]   # tuple of values state and cumalitive cost. Hene state[1] as it represents the  and state[0] the cost
      
        if problem.isGoalState(state[1]): # found the goal!!!
            print("Found Goal")
            while backtrack != problem.getStartState():  # action logic management
                actions.append(dict[backtrack][1])  # same as bfs and dfs backtrack up the map
                backtrack = dict[backtrack][0]
            actions.reverse()
            return actions

        closedtSet.append(state[1])
      

        for (next_state, action, cost) in problem.getSuccessors(state[1]):
            if next_state not in closedtSet:  # only add the children if tey have not been visited
                if next_state not in [str(i[2]) for i in openSet]:
                
                    heappush(openSet, ((cost+state[0]), next_state))

                if (next_state not in dict.keys()): # add the path to the map
                    dict[next_state] = (state[1], action, cost)

                # elif next_state in dict.keys() and ((cost+state[1]) < dict[next_state][2] ):

                elif (cost + state[0]) < dict[next_state][2]:  #update the path in the map if the child is already in the fringe and had its path added to the map
                                                                # Howver, the new path to this child has a lower cost. Thus, the path of the child needs to be modified. 
                    dict[next_state] = (state[1], action, cost + state[0])
          

    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """
    YOUR CODE HERE
    """
    i = 0
    closedtSet = []  # keeps track of the nodes that have been visited
    openSet = []  # keeps track of the children of the node
    h_start = heuristic(problem.getStartState(), problem)
    # openset has the values (f value, g value, state)
    heappush(openSet, (h_start, 0, problem.getStartState())) # now the openset has the tuple((f(n)+g(n), f(n), state)
    

    dict = {}  # map to trace back the actions thar are going to pass to the search agent
    actions = []  # the list of actions

    i = 0
    while len(openSet) > 0:
      
        f, g, state = heappop(openSet)  

        closedtSet.append((f, state)) # as the value is removed immideatly add it to the clsoedlist 
        
        if problem.isGoalState(state):
            backtrack = state
            while backtrack != problem.getStartState():  # action logic management
           
                actions.append(dict[backtrack][1]) # just like DFS,BFS, UCS backtrack up the map
                backtrack = dict[backtrack][0]
            actions.reverse()
            return actions

        for (next_state, action, cost) in problem.getSuccessors(state):
  
            child_g = cost+g # calculate the cost for the child 
            child_h = heuristic(next_state, problem) # calculate the hurisitn
            child_f = child_g + child_h  # calculate the sum 
            if next_state in [i[1] for i in closedtSet]: # if the child is in the closed set the skip

                continue
            
            # if next state is in open set
            if next_state in [i[3] for i in openSet]: # if child is in the openset then skip as according to the optimality of A* the current child in the fringe
                                                     # is the most optimal path unlike UCS wheere changes to the path need to be made
                if ([i for i in openSet if i[3] == next_state][0][2]) <= child_g:
                    # print("booo")
                    
                    continue
            
            heappush(openSet, (child_f, child_g, next_state)) 

            if next_state not in [i[1] for i in closedtSet]:  # make sure that the child is not in the closedset to avoid loops inside the path
                dict[next_state] = (state, action, child_f)
    
    util.raiseNotDefined()



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
