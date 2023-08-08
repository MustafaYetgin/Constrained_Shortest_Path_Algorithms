import constrained_genetic_algorithm
import matplotlib.pyplot as plt

backend = 'TkAgg'
plt.switch_backend(backend)
def draw_grid(shortest_paths_in_each_generation, grid_width, grid_height, obstacles, launch_point, target):


    plt.ion()
    plt.figure(figsize=(grid_width, grid_height))
    for x in range(grid_width + 1):
        plt.axvline(x, color='k', linestyle='--')

    for y in range(grid_height + 1):
        plt.axhline(y, color='k', linestyle='--')

    for obstacle in obstacles:
        one_corner_x = int(obstacle[0])
        one_corner_y = int(obstacle[1])
        x_square = [one_corner_x, one_corner_x, one_corner_x+1, one_corner_x+1]
        y_square = [one_corner_y, one_corner_y+1, one_corner_y+1 , one_corner_y]
        plt.fill(x_square,y_square,color='blue', alpha=1)

    for shortest_path in shortest_paths_in_each_generation:
        for point in shortest_path:
            plt.plot([point[0]], [point[1]], marker='o', markersize=10, color='pink')
        for i in range(len(shortest_path) - 1):
            x1, y1 = shortest_path[i]
            x2, y2 = shortest_path[i + 1]
            plt.plot([x1, x2], [y1, y2], color='k', linewidth = 4)
        #plt.pause(0.1)

    for point in shortest_paths_in_each_generation[99]:
        plt.plot([point[0]], [point[1]], marker='o', markersize=10, color='pink')
    for i in range(len(shortest_path) - 1):
        x1, y1 = shortest_path[i]
        x2, y2 = shortest_path[i + 1]
        plt.plot([x1, x2], [y1, y2], color='green', linewidth=5)

    plt.plot([launch_point[0]], [launch_point[1]], marker='o', markersize=10, color='none',markeredgecolor='purple')

    plt.plot([target[0]], [target[1]], marker='o', markersize=10, color='r')

    plt.axis('scaled')
    plt.axis('off')
    plt.ioff()

def generate_path_from_given_angles(launch,target,angles_array):

    path = []
    previous_point = (launch[0]-1,launch[1])
    current_point = launch
    path.append(launch)

    for angle in angles_array:

        temp = current_point

        straight_point = genetic_algorithm.find_point_straight_on_circle(current_point,previous_point)

        current_point = genetic_algorithm.turn_point(straight_point, angle, current_point)

        previous_point = temp

        path.append(current_point)

    path.append(target)

    return path

bests_in_each_generation = constrained_genetic_algorithm.genetic_algorithm([25,25],[40,40],[(30,30),(29,31)])

shortest_paths_in_each_generation = []
for best in bests_in_each_generation:

    shortest_paths_in_each_generation.append(generate_path_from_given_angles([25,25],[40,40],best.way_points))

draw_grid(shortest_paths_in_each_generation,100,100,[(30,30),(31,29),(29,31)],[25,25],[40,40])

plt.show()


