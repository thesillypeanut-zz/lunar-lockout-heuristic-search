#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the LunarLockout  domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from lunarlockout import LunarLockoutState, Direction, lockout_goal_state #for LunarLockout specific classes and problems

#LunarLockout HEURISTICS
def heur_trivial(state):
    '''trivial admissible LunarLockout heuristic'''
    '''INPUT: a LunarLockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
    return 0

def heur_manhattan_distance(state):
  sum_L_dist = 0
  center = int((state.width-1)/2)
  xanadus = ((state.xanadus[0], state.xanadus[1]),) if isinstance(
    state.xanadus[0], int) else state.xanadus

  for xanadu in xanadus:

    if xanadu[0] == center and xanadu[1] == center:
      continue

    elif xanadu[0] == center:
      sum_L_dist += abs(center - xanadu[1])

    elif xanadu[1] == center:
      sum_L_dist += abs(center - xanadu[0])

    else:
      sum_L_dist += abs(center - xanadu[1])
      sum_L_dist += abs(center - xanadu[0])

  return sum_L_dist

def heur_L_distance(state):
  #IMPLEMENT
    '''L distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''

    sum_L_dist = 0
    center = int((state.width-1)/2)
    xanadus = ((state.xanadus[0], state.xanadus[1]),) if isinstance(state.xanadus[0], int) else state.xanadus

    for xanadu in xanadus:

      if xanadu[0] == center and xanadu[1] == center:
        continue

      elif xanadu[0] == center or xanadu[1] == center:
        sum_L_dist += 1

      else:
        sum_L_dist += 2

    return sum_L_dist

def heur_alternate(state):
    #IMPLEMENT
    '''a better lunar lockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''

    # This is an extension of the L-distance heuristic and takes rewards and penalties into account
    # along the L-path. Rewards decrease the h-value while penalties increase it. The method returns
    # the sum of h-values for all xanadus.
    #
    # For each xanadu, it first checks whether it is at any of the four corners of the board, away
    # from all robots. If it is, the method terminates and returns a large number since such a corner
    # xanadu cannot be guided to the exit.
    #
    # If it is not a corner xanadu, h-value is calculated along the L-path from the xanadu to the
    # exit. H-value calculation for the L-path is broken down into vertical and horizontal subpaths
    # and then added together. Two L-distances are calculated for a xanadu that does not have the
    # same x or y coordinates as the exit and the max is taken. For every robot encountered on a
    # subpath, L-distance is increased by one to account for the additional move required for the
    # robot to move out of the way. However, if there is another xanadu at the opposite end of the
    # board, along the same horizontal/vertical line but not directly on the subpath, we can assume
    # that this xanadu will help remove the robot first and so L-distance is not increased. Lastly,
    # L-distance is increased by one if there is a robot at the end of the subpath to help guide
    # the xanadu along this subpath, two otherwise.

    def _get_vert_L_dist(xanadu_x, xanadu_y):
      start_y = xanadu_y + 1 if center > xanadu_y else center
      end_y = xanadu_y if center < xanadu_y else center + 1
      l_dist = 0

      for y in range(start_y, end_y):
        if (xanadu_x, y) in robots:
          l_dist += 1

      start_y = center if center > xanadu_y else 0
      end_y = center if center < xanadu_y else width
      for y in range(start_y, end_y):
        if (xanadu_x, y) in xanadus and l_dist > 0:
          l_dist -= 1

      helper_y = center + 1 if center > xanadu_y else center - 1
      l_dist += 1 if (xanadu_x, helper_y) in robots else 2

      return l_dist

    def _get_horiz_L_dist(xanadu_x, xanadu_y):
      start_x = xanadu_x + 1 if center > xanadu_x else center
      end_x = xanadu_x if center < xanadu_x else center + 1
      l_dist = 0

      for x in range(start_x, end_x):
        if (x, xanadu_y) in robots:
          l_dist += 1

      start_x = center if center > xanadu_x else 0
      end_x = center if center < xanadu_x else width
      for x in range(start_x, end_x):
        if (x, xanadu_y) in xanadus and l_dist > 0:
          l_dist -= 1

      helper_x = center + 1 if center > xanadu_x else center - 1
      l_dist += 1 if (helper_x, xanadu_y) in robots else 2

      return l_dist

    def _is_corner_xanadu(xanadu):
      bot_left_xanadu = False
      bot_right_xanadu = False
      top_left_xanadu = False
      top_right_xanadu = False
      for robot in robots:
        if robot[0] > xanadu[0] and robot[1] > xanadu[1]:
          bot_left_xanadu = True
          bot_right_xanadu = False
          top_left_xanadu = False
          top_right_xanadu = False
        elif robot[0] < xanadu[0] and robot[1] < xanadu[1]:
          bot_left_xanadu = False
          bot_right_xanadu = False
          top_left_xanadu = False
          top_right_xanadu = True
        elif robot[0] > xanadu[0] and robot[1] < xanadu[1]:
          bot_left_xanadu = False
          bot_right_xanadu = False
          top_left_xanadu = True
          top_right_xanadu = False
        elif robot[0] < xanadu[0] and robot[1] > xanadu[1]:
          bot_left_xanadu = False
          bot_right_xanadu = True
          top_left_xanadu = True
          top_right_xanadu = False

      return bot_left_xanadu or bot_right_xanadu or top_left_xanadu or top_right_xanadu


    sum_L_dist = 0
    width = state.width
    center = int((width-1)/2)
    xanadus = ((state.xanadus[0], state.xanadus[1]),) if isinstance(state.xanadus[0], int) else state.xanadus
    robots = ((state.robots[0], state.robots[1]),) if isinstance(state.robots[0], int) else state.robots
    robots = set(robots)

    for xanadu in xanadus:
      # If xanadu is in a corner, check if it has helper robots around it
      corner_xanadu = _is_corner_xanadu(xanadu)
      if corner_xanadu:
        helper_robot = False
        for robot in robots:
          if robot[0] == xanadu[0] or robot[1] == xanadu[1]:
            helper_robot = True
            break
        # Return a high number if there are no helper robots to guide it
        if not helper_robot:
          return 10000000

      # L-distance is 0 for xanadus that are directly on the exit
      if xanadu[0] == center and xanadu[1] == center:
        continue

      # Calculate only vertical L-distance for xanadus with same x coordinate as exit
      elif xanadu[0] == center:
        sum_L_dist += _get_vert_L_dist(xanadu[0], xanadu[1])

      # Calculate only horizontal L-distance for xanadus with same y coordinate as exit
      elif xanadu[1] == center:
        sum_L_dist += _get_horiz_L_dist(xanadu[0], xanadu[1])

      else:
        # Calculate two different L-distances for xanadus that don't share any coordinate
        # with the exit and take the max of the two
        l1 = _get_vert_L_dist(xanadu[0], xanadu[1]) + _get_horiz_L_dist(xanadu[0], center)
        l2 = _get_horiz_L_dist(xanadu[0], xanadu[1]) + _get_vert_L_dist(center, xanadu[1])

        sum_L_dist += max(l1, l2)

    return sum_L_dist


def fval_function(sN, weight):
  #IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

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
    return sN.gval + weight*sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=4., timebound = 1):
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  search_stop_time = os.times()[0] + timebound
  se = SearchEngine('custom')

  wrapped_fval_function = (lambda sN: fval_function(sN, weight))
  se.init_search(initial_state, goal_fn=lockout_goal_state,
                 heur_fn=heur_fn, fval_function=wrapped_fval_function)
  final = se.search(timebound)
  weight -= 1

  while weight > 1 and os.times()[0] < search_stop_time:
    new_final = se.search(search_stop_time - os.times()[0])
    final = new_final if new_final else final
    weight -= 1

  return final if final else False


def anytime_gbfs(initial_state, heur_fn, timebound = 1):
  #IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  return 0

PROBLEMS = (
  #5x5 boards: all are solveable
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 1))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 2))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 3))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 1))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 2))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 3))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 4))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 0))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 1))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (0, 2),(0,4),(2,0),(4,0)),((4, 4))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 0))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 1))),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 3))),
  #7x7 BOARDS: all are solveable
  LunarLockoutState("START", 0, None, 7, ((4, 2), (1, 3), (6,3), (5,4)), ((6, 2))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (4, 2), (2,6)), ((4, 6))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (3, 1), (4, 1), (2,6), (4,6)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((1, 2), (0 ,2), (2 ,3), (4, 4), (2, 5)), ((2, 4),(3, 1),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 2), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 1), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (0 ,2), (1 ,2), (6, 4), (2, 5)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 9, ((2, 2), (3, 4), (4, 5), (5, 5),
                                          (5, 0), (6, 1), (6, 4), (7, 0), (8, 1), (8, 3)), ((0, 0), (0, 8))),
  )

if __name__ == "__main__":

  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 1; #1 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(len(PROBLEMS)): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    print("*******RUNNING A STAR*******") 
    se = SearchEngine('astar', 'full')
    se.init_search(s0, lockout_goal_state, heur_L_distance)
    final = se.search(timebound) 

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime Weighted A-star")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]  
    weight = 4
    final = anytime_weighted_astar(s0, heur_L_distance, weight, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime GBFS")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]  
    final = anytime_gbfs(s0, heur_L_distance, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************")   



  

