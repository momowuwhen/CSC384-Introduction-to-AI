#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    sum = 0
    for box in state.boxes:
      min_distance = 0
      for storage in state.storage:
        curr_distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        if curr_distance < min_distance or min_distance == 0:
          min_distance = curr_distance
      sum = sum + min_distance
    return sum

#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

# check if a storage is occupied, return a boolean
def storage_occupied(state, storage):
    for box in state.boxes:
        if(box == storage):
            return True
    return False

# check if a box is in a storage, return a boolean
def box_in_storage(state, box):
    if box in state.storage:
        return True
    else:
        return False

# return a list of avaliable storages for a box
# if the box is already in storage, just return its current storage place
def avaliable_storages(state, box):
    if box_in_storage(state, box):
        return [box]
    else:
        avaliable_storage = []
        for storage in state.storage:
            if not storage_occupied(state, storage):
                avaliable_storage.append(storage) 
        return avaliable_storage

def deadlocks(state, box):
    left_border = 0
    right_border = state.width - 1
    top_border = 0
    bottom_border = state.height - 1
    x = box[0]
    y = box[1]
    storages = state.storage
    horizontal_storages = []
    vertical_storages = []
    for storage in storages:
        horizontal_storages.append(storage[0])
        vertical_storages.append(storage[1])

    boxes = state.boxes
    obstacles = state.obstacles
    blocks = boxes.union(obstacles)
    # box cannot move at 4 corners and corners are not storage place either
    if x == left_border and y == top_border and ((left_border, top_border) not in storages):
        return True
    if x == left_border and y == bottom_border and ((left_border, bottom_border) not in storages):
        return True
    if x == right_border and y == top_border and ((right_border, top_border) not in storages):
        return True
    if x == right_border and y == bottom_border and ((right_border, bottom_border) not in storages):
        return True
    
    # box cannot reach storage if it can only move along edge but there's no storage on that edge
    if x == left_border and (left_border not in horizontal_storages):
        return True
    if x == right_border and (right_border not in horizontal_storages):
        return True
    if y == top_border and (top_border not in vertical_storages):
        return True
    if y == bottom_border and (bottom_border not in vertical_storages):
        return True

    # box cannot move if box/obstacle and edge restrict horizontal and vertical movements
    if x == left_border and \
    (((x, y+1) in blocks) or ((x, y-1) in blocks)):
        return True
    if x == right_border and \
    (((x, y+1) in blocks) or ((x, y-1) in blocks)):
        return True
    if y == top_border and \
    (((x+1, y) in blocks) or ((x-1, y) in blocks)):
        return True
    if y == bottom_border and \
    (((x+1, y) in blocks) or ((x-1, y) in blocks)):
        return True
    
    # box cannot move if there are box/obstacle next to box that restrict horizontal and vertical movement 
    if ((x-1, y) in blocks) and ((x, y+1) in blocks):
        return True
    if ((x-1, y) in blocks) and ((x, y-1) in blocks):
        return True
    if ((x+1, y) in blocks) and ((x, y+1) in blocks):
        return True
    if ((x+1, y) in blocks) and ((x, y-1) in blocks):
        return True
    
    # box cannot reach storage if in a tunnel without storage
    # length 2
    if x+1 == right_border:
        if ((x, y+1) in blocks) and ((x, y-1) in blocks):
            if ((x+1, y+1) in blocks) or ((x+1, y-1) in blocks):
                return True
    
    if x-1 == left_border:
        if ((x, y+1) in blocks) and ((x, y-1) in blocks):
            if ((x-1, y+1) in blocks) or ((x-1, y-1) in blocks):
                return True

    if y+1 == top_border:
        if ((x-1, y) in blocks) and ((x+1, y) in blocks):
            if ((x-1, y+1) in blocks) or ((x+1, y+1) in blocks):
                return True
    if y-1 == bottom_border:
        if ((x-1, y) in blocks) and ((x+1, y) in blocks):
            if ((x-1, y-1) in blocks) or ((x+1, y-1) in blocks):
                return True
    # length 3
    if x+2 == right_border:
        if ((x, y+1) in blocks) and ((x, y-1) in blocks):
            if ((x+2, y+1) in blocks) and ((x+2, y-1) in blocks):
                if ((x+1, y+1) in blocks) or ((x+1, y-1) in blocks):
                    return True
    
    if x-2 == left_border:
        if ((x, y+1) in blocks) and ((x, y-1) in blocks):
            if ((x-2, y+1) in blocks) and ((x-2, y-1) in blocks):
                if ((x-1, y+1) in blocks) or ((x-1, y-1) in blocks):
                    return True

    if y+2 == top_border:
        if ((x-1, y) in blocks) and ((x+1, y) in blocks):
            if ((x-1, y+2) in blocks) and ((x+1, y+2) in blocks):
                if ((x-1, y+1) in blocks) or ((x+1, y+1) in blocks):
                    return True
    if y-2 == bottom_border:
        if ((x-1, y) in blocks) and ((x+1, y) in blocks):
            if ((x-1, y-2) in blocks) and ((x+1, y-2) in blocks):
                if ((x-1, y-1) in blocks) or ((x+1, y-1) in blocks):
                    return True
    # length 4
    if x+3 == right_border:
        if ((x, y+1) in blocks) and ((x+1, y+1) in blocks) and ((x+2, y+1) in blocks) and ((x+3, y+1) in blocks):
            if ((x, y-1) in blocks) and ((x+1, y-1) in blocks) and ((x+2, y-1) in blocks) and ((x+3, y-1) in blocks):
                return True
    
    if x-3 == left_border:
        if ((x, y+1) in blocks) and ((x-1, y+1) in blocks) and ((x-2, y+1) in blocks) and ((x-3, y+1) in blocks):
            if ((x, y-1) in blocks) and ((x-1, y-1) in blocks) and ((x-2, y-1) in blocks) and ((x-3, y-1) in blocks):
                return True

    if y+3 == top_border:
        if ((x-1, y) in blocks) and ((x-1, y+1) in blocks) and ((x-1, y+2) in blocks) and ((x-1, y+3) in blocks):
            if ((x+1, y) in blocks) and ((x+1, y+1) in blocks) and ((x+1, y+2) in blocks) and ((x+1, y+3) in blocks):
                return True

    if y-3 == bottom_border:
        if ((x-1, y) in blocks) and ((x-1, y-1) in blocks) and ((x-1, y-2) in blocks) and ((x-1, y-3) in blocks):
            if ((x+1, y) in blocks) and ((x+1, y-1) in blocks) and ((x+1, y-2) in blocks) and ((x+1, y-3) in blocks):
                return True
    # length 5
    if x+4 == right_border:
        if ((x, y+1) in blocks) and ((x+1, y+1) in blocks) and ((x+2, y+1) in blocks) and ((x+3, y+1) in blocks) and ((x+4, y+1) in blocks):
            if ((x, y-1) in blocks) and ((x+1, y-1) in blocks) and ((x+2, y-1) in blocks) and ((x+3, y-1) in blocks) and ((x+4, y-1) in blocks):
                return True
    
    if x-4 == left_border:
        if ((x, y+1) in blocks) and ((x-1, y+1) in blocks) and ((x-2, y+1) in blocks) and ((x-3, y+1) in blocks) and ((x-4, y+1) in blocks):
            if ((x, y-1) in blocks) and ((x-1, y-1) in blocks) and ((x-2, y-1) in blocks) and ((x-3, y+1) in blocks) and ((x-4, y+1) in blocks):
                return True

    if y+4 == top_border:
        if ((x-1, y) in blocks) and ((x-1, y+1) in blocks) and ((x-1, y+2) in blocks) and ((x-1, y+3) in blocks) and ((x-1, y+4) in blocks):
            if ((x+1, y) in blocks) and ((x+1, y+1) in blocks) and ((x+1, y+2) in blocks) and ((x+1, y+3) in blocks) and ((x+1, y+4) in blocks):
                return True

    if y-4 == bottom_border:
        if ((x-1, y) in blocks) and ((x-1, y-1) in blocks) and ((x-1, y-2) in blocks) and ((x-1, y-3) in blocks) and ((x-1, y-4) in blocks):
            if ((x+1, y) in blocks) and ((x+1, y-1) in blocks) and ((x+1, y-2) in blocks) and ((x+1, y-3) in blocks) and ((x+1, y-4) in blocks):
                return True

    return False

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    
    res = 0
    # check if it's a deadlock first
    for box in state.boxes:
        if not box_in_storage(state, box) and deadlocks(state, box):
            return float("inf")
    
    # cost from robot to box
    for robot in state.robots:
        min_robot_box = 0
        for box in state.boxes:
            curr_robot_box = abs(robot[0] - box[0]) + abs(robot[1] - box[1])
            if (min_robot_box == 0) or (curr_robot_box < min_robot_box):
                min_robot_box = curr_robot_box
        res = res + min_robot_box
    
    # cost from box to storage
    for box in state.boxes:
        min_box_storage = 0
        for storage in avaliable_storages(state, box):
            curr_box_storage = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
            if (min_box_storage == 0) or (curr_box_storage < min_box_storage):
                min_box_storage = curr_box_storage
        res = res + min_box_storage
    
    return res    
    
def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the standard form of weighted A* (i.e. g + w*h)

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    
    fval = sN.gval + (weight * sN.hval)
    return fval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of anytime weighted astar algorithm'''
    
    start_time = os.times()[0]
    end_time = start_time + timebound

    se = SearchEngine('custom', 'full')

    costbound = float("inf") 
    time_remaining = end_time - os.times()[0]
    
    result = False
    while time_remaining > 0: 
        wrapped_fval_function = (lambda sN: fval_function(sN, weight))
        se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
        final = se.search(time_remaining, (float("inf"), float("inf"), costbound))[0] 
        weight = weight / 1.2
        if not final:
            break
        time_remaining = end_time - os.times()[0]
        costbound = final.gval + heur_fn(final)
        result = final
        time_remaining = end_time - os.times()[0]
    return result

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of anytime greedy best-first search'''
  
    end_time = os.times()[0] + timebound

    se = SearchEngine("best_first", "full")
    se.init_search(initial_state, sokoban_goal_state, heur_fn)

    result = False
    costbound = float("inf")

    time_remaining = end_time - os.times()[0]
    while time_remaining > 0:
        final_state = se.search(time_remaining, (costbound, float('inf'), float('inf')))[0]
        if not final_state:
            break
        result = final_state
        costbound = result.gval
        time_remaining = end_time - os.times()[0]
    return result
