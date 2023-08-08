import math
import sys
import numpy as np
import matplotlib.pyplot as plt


def draw_grid(shortest_path, grid_width, grid_height, obstacles, launch_point, target,visited,all_neigbours):

    plt.ion()
    plt.figure(figsize=(grid_width, grid_height))
    for x in range(grid_width + 1):
        plt.axvline(x, color='k', linestyle='--')

    for y in range(grid_height + 1):
        plt.axhline(y, color='k', linestyle='--')

    for obstacle in obstacles:
        plt.plot([obstacle[1] ], [obstacle[0] ], marker='^', markersize=10, color='b')

    plt.plot([launch_point[1] ], [launch_point[0]], marker='o', markersize=10, color='none',
             markeredgecolor='purple')

    plt.plot([target[1] ], [target[0] ], marker='o', markersize=10, color='r')

    index = 0

    for unit in visited:

        plt.plot([unit[1]], [unit[0] ], marker='*', markersize=10, color='green')

        for unit in all_neigbours[index]:

            plt.plot([unit[1] ], [unit[0] ], marker='*', markersize=10, color='yellow')

            plt.pause(0.00001)

        index = index + 1

    for i in range(len(shortest_path) - 1):
        y1, x1 = shortest_path[i]
        y2, x2 = shortest_path[i + 1]
        plt.plot([x1 , x2 ], [y1 , y2], color='r')
        plt.pause(0.00001)

    plt.axis('scaled')
    plt.axis('off')
    plt.ioff()

def index_finder(row, column, column_num):

    return row*column_num + column

def find_closest(distances,column_num,available_neighbors):

    min_val = sys.maxsize

    min_val_row = sys.maxsize

    min_val_column = sys.maxsize

    for row, column in available_neighbors:

        curr_index = index_finder(row, column, column_num)

        if distances[curr_index] < min_val:

            min_val = distances[curr_index]

            min_val_column = column

            min_val_row = row

    return min_val_row, min_val_column




def update_neighbors(distances, row, column, row_num, column_num, parents, is_passed, available_neighbours) :

    update_patterns = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [-1, -1], [-1, 1], [1, -1]]

    for i in range(8):

        row_add = update_patterns[i][0]

        column_add = update_patterns[i][1]

        update_column = column + column_add

        update_row = row + row_add

        if row_num > update_row >= 0 and column_num > update_column >= 0 and not(is_passed[index_finder(update_row, update_column,column_num)]):

            if not([update_row, update_column] in available_neighbours):

                available_neighbours.append([update_row,update_column])

            possible_value = 0

            if i < 4:

                possible_value = distances[index_finder(row,column, column_num)] + 1
            else:

                possible_value = distances[index_finder(row,column,column_num)] + math.sqrt(2)

            if possible_value < distances[index_finder(update_row,update_column,column_num)] :

                distances[index_finder(update_row, update_column, column_num)] = possible_value

                parents[index_finder(update_row,update_column,column_num)] = [row, column]




def find_shortest_path(launch_point_coordinate, target_coordinate, obstacles_coordinates, row_num, column_num):

    size_of_grid = row_num * column_num

    parents = [[0,0]] * size_of_grid

    available_neighbors = []

    parents[index_finder(launch_point_coordinate[0],launch_point_coordinate[1],column_num)] = [-1,-1]

    distances = [sys.maxsize] * size_of_grid

    distances[index_finder(launch_point_coordinate[0],launch_point_coordinate[1],column_num)] = 0

    available_neighbors.append(launch_point_coordinate)

    is_passed = [False] * size_of_grid

    for row,column in obstacles_coordinates :

        if(not([row,column] == [target_coordinate[0],target_coordinate[1]]) ):

            is_passed[index_finder(row,column,column_num)] = True

    while not(is_passed[index_finder(target_coordinate[0],target_coordinate[1],column_num)]):

        row, column = find_closest(distances, column_num,available_neighbors)

        if (row,column) == (sys.maxsize,sys.maxsize) :

            return -45, -45

        is_passed[index_finder(row,column,column_num)] = True

        available_neighbors.remove([row,column])

        update_neighbors(distances, row, column, row_num, column_num,parents, is_passed, available_neighbors)



    return distances[index_finder(target_coordinate[0], target_coordinate[1], column_num)], parents



launch_point = [1,1]

target = [9,9]

#obstacles = [[1,5],[3,1],[3,2],[7,8],[9,5],[6,5],[3,4],[8,6],[5,8],[7,6],[3,3],[9,8],[8,9],[7,7],[1,3],[4,6],[0,1],[7,9],[6,3]]

obstacles = [[3,4],[2,4],[4,4],[4,3],[1,4],[4,2]]

grid_width = 10  # Number of cells in the width of the grid

grid_height = 10  # Number of cells in the height of the grid

obstaclesCheck = True

for obstacle in obstacles:
    if obstacle == target or obstacle == launch_point:
        obstaclesCheck = False

if grid_width<0 or grid_height<0 or not(0 <= launch_point[0] < grid_height) or not(0 <= launch_point[1] < grid_width) or not(0 <= target[0] < grid_height) or not(0 <= target[1] < grid_width) or not(obstaclesCheck):
    print("invalid input!!")

else:

    x,parents = find_shortest_path(launch_point,target,obstacles,grid_height,grid_width)

    path_to_target = []

    if (x,parents) == (-45,-45):

        path_to_target.append(launch_point)

        print("There is no way to reach the target!!!!")

    else:

        path_to_target.append(target)

        print("total distance elapsed : ", x)

        curr_index = index_finder(target[0],target[1],grid_width)

        print("finishing....")

        print([target[0],target[1]],"↑")

        while parents[curr_index] != [-1,-1] :

            path_to_target.append(parents[curr_index])

            print(parents[curr_index],"↑")

            curr_index = index_finder(parents[curr_index][0],parents[curr_index][1],grid_width)

        print("starting....")

    # Create a 2D array representing the grid
    grid = np.zeros((grid_width, grid_height))

    draw_grid(path_to_target,50,50,[(27,27),(26,27),(27,26)],(25,25),(35,35,))

    plt.show()

