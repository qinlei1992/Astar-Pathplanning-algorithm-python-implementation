# Credit for this: Nicholas Swift
# as found at https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
from warnings import warn
import numpy as np
import pickle
import matplotlib.pyplot as plt
import math
import time
import heapq


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.open = True
        self.valid = True

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __repr__(self):
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
        return self.f < other.f

    # defining greater than for purposes of heap queue
    def __gt__(self, other):
        return self.f > other.f


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, allow_diagonal_movement=False):
    """
    Returns a list of tuples as a path from the given start to the given end in the given maze
    """

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    explored_nodes = {}

    # Heapify the open_list and Add the start node
    heapq.heapify(open_list)
    heapq.heappush(open_list, start_node)

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while open_list:
        # Get the current node
        current_node = heapq.heappop(open_list)
        current_node.open = False
        explored_nodes[current_node.position] = current_node
        # closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            return return_path(current_node)

        for new_position in adjacent_squares:  # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            within_range_criteria = [
                node_position[0] > (maze.shape[0] - 1),
                node_position[0] < 0,
                node_position[1] > (maze.shape[1] - 1),
                node_position[1] < 0,
            ]

            if any(within_range_criteria):
                continue

            # Make sure walkable terrain
            if maze[node_position] != 0:
                continue

            neighbor = explored_nodes.get(node_position)

            tentative_g = current_node.g + 1

            if neighbor is None:
                new_node = Node(current_node, node_position)
                new_node.g = tentative_g
                new_node.h = abs(new_node.position[0] - end_node.position[0]) + abs(new_node.position[1] - end_node.position[1])
                new_node.f = new_node.g + new_node.h
                explored_nodes[node_position] = new_node
                heapq.heappush(open_list, new_node)

            elif tentative_g > neighbor.g or not neighbor.open:
                continue

            else:
                new_node = Node(current_node, node_position)
                new_node.g = tentative_g
                new_node.h = abs(new_node.position[0] - end_node.position[0]) + abs(new_node.position[1] - end_node.position[1])
                new_node.f = new_node.g + new_node.h
                explored_nodes[node_position] = new_node
                heapq.heappush(open_list, new_node)
                neighbor.valid = False

        while open_list and not open_list[0].valid:
            heapq.heappop(open_list)
            print('removed!')


if __name__ == '__main__':


    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    maze = np.array(maze)
    start = (0, 0)
    end = (7, 6)

    s_time = time.time()
    path = astar(maze, start, end)
    print(path)
    print(time.time()-s_time)
