# TODO: Implement the global planner here
import heapq
import numpy as np

from collections import deque

UNEXPLORED = 0

def get_backtrack(node, parents : dict):
    res = [node]
    while parents.get(node) != None:
        node = parents.get(node)
        print("btracking ", node)
        res.append(node)
    return res[::-1]
    
def get_neighbors(pos, robot_map):
    map_width, map_height = len(robot_map[0]), len(robot_map)
    x, y = pos
    neighbors = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            new_pos = (x+i, y+j)
            if 0 <= new_pos[0] < map_width and 0 <= new_pos[1] < map_height:
                neighbors.append(new_pos)
    return neighbors

def search(startInPixels, robot_map, goal_check_fn):
    start = startInPixels
    
    q = deque()
    q.append(start)
    
    parents = dict()
    visited = dict()
    visited[start] = 1
    
    while q:
        pos = q.popleft()
        if goal_check_fn(pos):
            return get_backtrack(pos, parents)
        neighbors = get_neighbors(pos, robot_map)
        for neighbor in neighbors:
            if neighbor not in visited:
                parents[neighbor] = pos
                q.append(neighbor)
                visited[neighbor] = 1
    return []


def get_global_plan_to_unexplored(startInPixels, robot_map):
    """
    Given start and goal position in pixels, and also map of the environment
    plan a collision-free path from start to nearest unexplored area.
    Returns a list of (x,y) coordinates in pixel coordinates
    """
    startInPixels = tuple(int(i) for i in startInPixels)
    goal_check_fn = lambda pos : robot_map[pos[1]][pos[0]] == UNEXPLORED
    return search(startInPixels, robot_map, goal_check_fn)
    

def get_global_plan(startInPixels, goalInPixels, robot_map):
    """
    Given start and goal position in pixels, and also map of the environment
    plan a collision-free path from start to goal.
    Returns a list of (x,y) coordinates in pixel coordinates
    """
    startInPixels = tuple(int(i) for i in startInPixels)
    goalInPixels = tuple(int(i) for i in goalInPixels)
    goal_check_fn = lambda pos : pos == goalInPixels
    return search(startInPixels, robot_map, goal_check_fn)