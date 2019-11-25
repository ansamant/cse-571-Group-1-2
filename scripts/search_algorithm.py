#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
import rospy
from std_msgs.msg import String
import argparse
import time
import Queue
import copy
import sys
import os
from datetime import datetime
from math import sqrt
from action_server import RobotActionsServer
from mazeGenerator import *

rospy.init_node("search_algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Possible arguments = {bfs, ucs, gbfs, astar, dead_end_filling, floodfill,leftHand}. Default value is dead_end_filling.", metavar='dead_end_filling', action='store', dest='algorithm', default="dead_end_filling", type=str)
parser.add_argument('-c', help="Use custom heuristic function. No value needed.", action='store_true', dest='custom_heuristic')
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
mazeInfo = Maze(6, 0.5)
# books = mazeInfo.generate_blocked_edges(1, 32,  1, root_path)
mazeInfoCopy = copy.deepcopy(mazeInfo)
print "blocked_edges: ", mazeInfo.blocked_edges
robot_action_server = RobotActionsServer({}, root_path, 32)
def manhattan_distance(state1, state2):
    return abs(state1[0]-state2.x) + abs(state1[1]-state2.y)
    
def opt_manhattan_distance(state1, state2):
    dx = 2 * abs(state1[0]-state2.x)
    dy = 2 * abs(state1[1]-state2.y)
    dist = dx + dy
    if dx !=0 and dy != 0:  # Add extra 2 for the turn cost if it is not along the horizontal or vertical to the goal
        dist += 2
    return dist
    
def euclidean_distance(state1, state2):
    dx = state1[0] - state2.x
    dy = state1[1] - state2.y
    return sqrt(dx*dx + dy*dy)
    
def opt_euclidean(state1, state2):
    dx = state1[0] - state2.x
    dy = state1[1] - state2.y
    return dx*dx + dy*dy

def cust_heuristic(state1, state2):
    return opt_manhattan_distance(state1, state2)

def is_dead_end(current_state, exclude = [], ismove = False, is_compl = False):
    helper = robot_action_server
    isDeadEnd = True
    valid_state_list = []
    s = current_state
    for item in helper.get_successors(current_state):
        x, y = item["state"]["x"], item["state"]["y"]
        if item["cost"] >= 0 and item["action"] == "MoveF" and (x,y) not in exclude:
            isDeadEnd = False
            if is_compl:
                item["action"] = ["MoveF"]
                valid_state_list.append(item)
            else:
                valid_state_list.append(item["state"])
        
    if not ismove:
        state = copy.deepcopy(current_state)
        ori = state["orientation"]
        ori_cw, ori_ccw = "EAST", "EAST"
        
        if "EAST" in ori:
            ori_cw = "SOUTH"
            ori_ccw = "NORTH"
        elif "WEST" in ori:
            ori_cw = "NORTH"
            ori_ccw = "SOUTH"
        elif "NORTH" in ori:
            ori_cw = "EAST"
            ori_ccw = "WEST"
        else:
            ori_cw = "WEST"
            ori_ccw = "EAST"

        state["orientation"] = ori_cw
        for item in helper.get_successors(state):
            x, y = item["state"]["x"], item["state"]["y"]
            if item["cost"] >= 0 and item["action"] == "MoveF" and (x,y) not in exclude:
                isDeadEnd = False
                if is_compl:
                    item["action"] = ["TurnCW", "MoveF"]
                    valid_state_list.append(item)
                else:
                    valid_state_list.append(item["state"])
                
        state = copy.deepcopy(state)
        state["orientation"] = ori_ccw
        for item in helper.get_successors(state):
            x, y = item["state"]["x"], item["state"]["y"]
            if item["cost"] >= 0 and item["action"] == "MoveF" and (x,y) not in exclude:
                isDeadEnd = False
                if is_compl:
                    item["action"] = ["TurnCCW", "MoveF"]
                    valid_state_list.append(item)
                else:
                    valid_state_list.append(item["state"])
                
    if isDeadEnd:
        print("current state ", current_state, " isDeadEnd ", isDeadEnd, " valid_state_list ", valid_state_list)
    return isDeadEnd, valid_state_list

def find_dead_ends(current_state, visited):
    list = []
    helper = robot_action_server
    isDeadEnd, valid_state_list = is_dead_end(current_state)
    if (not isDeadEnd):
        for state in valid_state_list:
            if json.dumps(state) not in visited:
                visited.add(json.dumps(state))
                list.extend(find_dead_ends(state, visited))
    else:
        list.append(current_state)
    return list

def fill_dead_end(state, maze, exclude):
    print "filling dead end for state ", state
    x1, y1 = int(state["x"] * 2), int(state["y"] * 2)
    _, valid_state_list = is_dead_end(state)
    while len(valid_state_list) == 1:
        s = valid_state_list[0]
        if (x1, y1) != (int(s["x"] * 2), int(s["y"] * 2)):
            maze[x1][y1] = -1
            exclude.append((s["x"], s["y"]))
            x1, y1 = int(s["x"] * 2), int(s["y"] * 2)
            _, valid_state_list = is_dead_end(s, exclude)
        else:
            print " rotating at ", s
            _, valid_state_list = is_dead_end(s, exclude, True)
        print "next state", s, " valid state len ", len(valid_state_list)
    
def walkthrough_maze(state, maze):
    helper = robot_action_server
    goal_state = helper.terminal_state
    actions = []
    visited = set()
    print "goal state ", goal_state
    while((state["x"],state["y"]) != (goal_state["x"],goal_state["y"])):
        _,paths = is_dead_end(state, [], False, True)
        #print "init paths ", paths
        for item in paths:
            x,y = int(item["state"]["x"] * 2), int(item["state"]["y"] * 2)
            if maze[x][y] == -1:
                paths.remove(item)
            elif item["cost"] < 0:
                paths.remove(item)
            elif (x,y) in visited:
                paths.remove(item)
            visited.add((x,y))
        if len(paths) != 1:
            print "State ", state," has ", len(paths), " paths"
        s = paths[0]
        print " chosen ", s
        actions.extend(s["action"])
        state = s["state"]

    print "mostly reached goal state ", state
    return actions

def dead_end_filling(use_custom_heuristic):
    helper = robot_action_server
    init_state = helper.current_state
    goal_state = helper.terminal_state
    print"\n\n\n-----goal state", goal_state
    dim = helper.maze_dim
    #possible_actions = helper.get_actions() 
    action_list = []
    dead_ends = []
    visited = set()
    
    dead_ends = find_dead_ends(init_state, visited)
    for i in dead_ends:
        if (i["x"],i["y"]) == (init_state["x"], init_state["y"]) or (i["x"],i["y"]) == (goal_state["x"], goal_state["y"]):
            print "removing ", i
            dead_ends.remove(i)
    print("dead_ends")
    print dead_ends
    maze = [[0 for i in range(dim[1])] for j in range(dim[0])]
    exclude = []
    for state in dead_ends:
        ori = state["orientation"]
        new_ori = "EAST"
        if ori == "NORTH":
            new_ori = "SOUTH"
        elif ori == "EAST":
            new_ori = "WEST"
        elif ori == "WEST":
            new_ori = "EAST"
        else:
            new_ori = "NORTH"
        state["orientation"] = new_ori
        fill_dead_end(state, maze, exclude)
        
    print("dead end filled")
    for row in maze:
        print row
        
    action_list = walkthrough_maze(init_state, maze)
    print "action list ", action_list
    return action_list
    
'''
Assumptions: Maze oriented in quadrant 1 for coordinate
param: mazeSize : int
returns: empty action list if goal state and a list of all moves to be made if otherwise
link: http://web.cecs.pdx.edu/~edam/Reports/2001/DWillardson.pdf?utm_source=rb-community&utm_medium=forum&utm_campaign=maze-solving-robot-flood-fill-algorithm-updated-w-video
'''


def floodfill(iter =1):
    helper = robot_action_server
    current = helper.current_state
    target = helper.terminal_state
    dim = helper.maze_dim
    # state_rep = {target: 0}
    # get matrix with values
    matrix = gen(dim, target)
    print(matrix)
    action_list = list()
    visited = list()
    while iter >= 0:

        while True:
            if (current["x"], current["y"]) == (target["x"], target["y"]):
                        break
            x_i = int(current["x"] * 2)
            y_i = int(current["y"] * 2)
            val_i = matrix[x_i][y_i]
            # get next states
            # assumes only valid
            dead_end, next_states = flood_orientation(current)
            # if you have reached a dead end, update cost and go back to start
            if dead_end:
                while dead_end:
                    matrix[x_i][y_i] = 100000000 # dont make a maze where this value doesnt work
                    if (x_i, y_i) == (0, 0):
                        print("Invalid State: this maze starts in a dead end!")
                        exit()
                    else:
                        ori = current["orientation"]
                        if ori == "NORTH":
                            new_ori = "SOUTH"
                        elif ori == "EAST":
                            new_ori = "WEST"
                        elif ori == "WEST":
                            new_ori = "EAST"
                        else:
                            new_ori = "NORTH"

                        current["orientation"] = new_ori
                        print("CURRENT", current)
                        dead_end, next_states = flood_orientation(current)
            print("Next_STATES", next_states)
            a_dict = {}
            for state in next_states:
                next_state, cost = state["state"], state["cost"]
                if cost < 0:
                    continue
                x = int(next_state["x"]*2)
                y = int(next_state["y"]*2)
                val = matrix[x][y]
                print("COST:", val)
                action = str(state["action"])
                a_dict[action] = (val, state["action"])
            next_move = min(a_dict.keys(), key=(lambda k: a_dict[k][0]))
            print("NEXT_MV", next_move)
            if a_dict[next_move][0] > val_i:
                matrix[x_i][y_i] = a_dict[next_move][0] + 1
            nxt = a_dict[next_move][1]
            print("NXT", nxt)
            if iter - 1 < 0:
                action_list.extend(nxt)
            for i in next_states:
                if i["action"] == nxt :
                    current = i["state"]
                    break
        # Now run with updated values


        current = helper.current_state
        iter -= 1
    return action_list


def gen(size, goal):
    x = int(goal["x"] * 2)
    y = int(goal["y"] * 2)
    result = [[0]*size[1] for a in range(size[0])]

    for i in range(len(result)):
        for j in range(len(result[i])):
            if (x == i) and (y == j):
                continue
            else:
                result[i][j] = flood_manhattan(x, i, y, j)
    return result


def flood_manhattan(x, x1, y, y1):
    return abs(x - x1) + abs(y - y1)


def flood_orientation(current_state, exclude=[], ismove=False, is_compl=True):
    helper = robot_action_server
    isDeadEnd = True
    s =current_state
    valid_state_list = []
    for item in helper.get_successors(current_state):
        print("ITEM1", item)
        x, y = item["state"]["x"], item["state"]["y"]
        if item["cost"] >= 0 and item["action"] == "MoveF" and (x, y) not in exclude:
            isDeadEnd = False
            if is_compl:
                item["action"] = ["MoveF"]
                valid_state_list.append(item)
            else:
                valid_state_list.append(item["state"])

    if not ismove:
        state = copy.deepcopy(current_state)
        ori = state["orientation"]
        ori_cw, ori_ccw = "EAST", "EAST"

        if "EAST" in ori:
            ori_cw = "SOUTH"
            ori_ccw = "NORTH"
        elif "WEST" in ori:
            ori_cw = "NORTH"
            ori_ccw = "SOUTH"
        elif "NORTH" in ori:
            ori_cw = "EAST"
            ori_ccw = "WEST"
        else:
            ori_cw = "WEST"
            ori_ccw = "EAST"

        state["orientation"] = ori_cw
        for item in helper.get_successors(state):
            print("ITEM2", item)
            x, y = item["state"]["x"], item["state"]["y"]
            if item["cost"] >= 0 and item["action"] == "MoveF" and (x, y) not in exclude:
                isDeadEnd = False
                if is_compl:
                    item["action"] = ["TurnCW", "MoveF"]
                    valid_state_list.append(item)
                else:
                    valid_state_list.append(item["state"])

        state = copy.deepcopy(state)
        state["orientation"] = ori_ccw
        for item in helper.get_successors(state):
            print("ITEM3", item)
            x, y = item["state"]["x"], item["state"]["y"]
            if item["cost"] >= 0 and item["action"] == "MoveF" and (x, y) not in exclude:
                isDeadEnd = False
                if is_compl:
                    item["action"] = ["TurnCCW", "MoveF"]
                    valid_state_list.append(item)
                else:
                    valid_state_list.append(item["state"])

    if isDeadEnd:
        print("current state ", current_state, " isDeadEnd ", isDeadEnd, " valid_state_list ", valid_state_list)
    return isDeadEnd, valid_state_list


def leftHand(use_custom_heuristic):
    helper = robot_action_server
    currState = helper.current_state
    goal_state = helper.terminal_state

    action_list = []

    while (not (currState["x"], currState["y"]) == (goal_state["x"], goal_state["y"])):

        for item in helper.get_successors(currState):
            flag = True
            if item["action"] == "TurnCCW" and item["cost"] >= 0:
                leftState = item["state"]
                subItems = helper.get_successors(leftState)
                for next in subItems:

                    if next["action"] == "MoveF" and next["cost"] >= 0:
                        currState = next["state"]
                        action_list.append("TurnCCW")
                        action_list.append("MoveF")
                        flag = False
                        break

                if (flag == False):
                    continue

            if (item["action"] == "MoveF" and item["cost"] >= 0):
                currState = item["state"]
                action_list.append("MoveF")
                continue

            for item2 in helper.get_successors(currState):
                if (item2["action"] == "TurnCW"):
                    currState = item2["state"]
                    action_list.append("TurnCW")

    return action_list

def exec_action_list(action_list):
    '''
    publishes the list of actions to the publisher topic
    action_list: list of actions to execute
    '''
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))


if __name__ == "__main__":
    # DO NOT MODIFY BELOW CODE
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    if args.algorithm in ["bfs", "ucs"] and args.custom_heuristic == True:
        print ("Error: "+args.algorithm+" called with heuristic")
        exit(1)

    start_time = time.time()
    actions = algorithm(args.custom_heuristic)
    time_taken = time.time() - start_time
    print("Time Taken = " + str(time_taken))
    print("Plan = " + str(actions))
    exec_action_list(actions)
