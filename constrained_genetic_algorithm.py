import random
import math
import sys

import numpy as np


# Define the problem constants
LAUNCH_POINT = (0, 0)  # Launch point coordinate (x, y)
TARGET_POINT = (10, 10)  # Target point coordinate (x, y)
OBSTACLE_POINTS = [(3, 5), (7, 8), (5, 2)]  # Obstacle points coordinates [(x1, y1), (x2, y2), ...]

# Genetic algorithm parameters


MUTPB = 0.2  # Mutation probability
GENERATIONS = 250



class Individual:

    def __init__(self):
        self.way_points = []
        self.penalty_value = 0


# Euclidean distance between two points
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def eucledian_distance(current_coord, target_coord):
    return math.sqrt(math.pow(target_coord[1]-current_coord[1], 2)+math.pow(target_coord[0]-current_coord[0], 2))

def can_hit_target(center, straight_point,target ,max_angle_deg):

    center_to_point = [straight_point[0]-center[0], straight_point[1]-center[1]]
    center_to_target = [target[0]-center[0], target[1]-center[1]]

    magnitude_center_to_point = eucledian_distance([center[0],center[1]],[straight_point[0],straight_point[1]])
    magnitude_center_to_target = eucledian_distance([center[0], center[1]],[target[0], target[1]])

    dot_product = center_to_point[0] * center_to_target[0] + center_to_point[1] * center_to_target[1]

    cos_angle = dot_product / (magnitude_center_to_point*magnitude_center_to_target)

    angle = math.acos(cos_angle)

    return -max_angle_deg <= angle <= max_angle_deg

def add_or_subtract(point,slope,parent):
    if slope < 0:
        if parent[1] > point[1]:
            return 1, -1
        else:
            return -1, 1
    elif slope > 0:
        if parent[1] > point[1]:
            return -1, -1
        else:
            return 1, 1
    else:
        if parent[0] > point[0]:
            return -1, 0
        else:
            return 1, 0

def find_point_straight_on_circle(center, parent):

    if center[0] - parent[0] == 0:

         if center[1] - parent[1] < 0:
             return [center[0], center[1] - 3.1]

         elif center[1] - parent[1] > 0:
             return [center[0], center[1] + 3.1]

         else:
             return [center[0], center[1]]

    else:

        slope = (center[1] - parent[1]) / (center[0] - parent[0])

        x_to_add = math.sqrt(math.pow(3.1, 2)/(math.pow(slope, 2)+1))

        y_to_add = abs(x_to_add*slope)

        mult1, mult2 = add_or_subtract(center, slope, parent)

        x_val = center[0] + x_to_add*mult1

        y_val = center[1] + y_to_add*mult2

        return [x_val, y_val]


def find_point_on_circle(center, radius, angle):

    cx, cy = center[0], center[1]

    x_axis = cx + radius*math.cos(angle)

    y_axis = cy + radius*math.sin(angle)

    return [x_axis, y_axis]

def is_cut_base(obstacle_integer_coordinates, center, node_to_go):

    bigger_y, smaller_y = center[1], node_to_go[1]
    bigger_x, smaller_x = center[0], node_to_go[0]

    if center[1] < node_to_go[1]:
        bigger_y = node_to_go[1]
        smaller_y = center[1]

    if center[0] < node_to_go[0]:
        bigger_x = node_to_go[0]
        smaller_x = center[0]

    cut = False

    if center[1] - node_to_go[1] == 0:

        if obstacle_integer_coordinates[1] <= center[1] <= obstacle_integer_coordinates[1] + 1:

            if smaller_x <= obstacle_integer_coordinates[0] <= bigger_x or smaller_x <= obstacle_integer_coordinates[0]+1 <= bigger_x:

                cut = True

    elif center[0] - node_to_go[0] == 0:

        if obstacle_integer_coordinates[0] <= center[0] <= obstacle_integer_coordinates[0] + 1:

            if smaller_y <= obstacle_integer_coordinates[1] <= bigger_y or smaller_y <= obstacle_integer_coordinates[1] + 1 <= bigger_y:

                cut = True

    else:

        slope = (node_to_go[1] - center[1])/(node_to_go[0] - center[0])

        b_value = center[1] - slope*center[0]

        possible_y_value1 = slope*obstacle_integer_coordinates[0] + b_value

        possible_y_value2 = slope*(obstacle_integer_coordinates[0]+1) + b_value

        possible_x_value1 = (obstacle_integer_coordinates[1] - b_value) / slope
        possible_x_value2 = (obstacle_integer_coordinates[1] + 1 - b_value) / slope

        # if abs(node_to_go.column_val) - 27.610262 < 0.000001 and abs(node_to_go.row_val - 16.302274) < 0.000001 and abs(center.column_val - 27.610262) < 0.000001 and obstacle_integer_coordinates == [27,16]:
        #     print(smaller_x," < ", possible_x_value2, " < ", bigger_x,"   SLOPE: ",slope)
        if ((obstacle_integer_coordinates[1] <= possible_y_value1 <= obstacle_integer_coordinates[1] + 1 and smaller_y <= possible_y_value1 <= bigger_y) or (obstacle_integer_coordinates[1] <= possible_y_value2 <= obstacle_integer_coordinates[1] + 1 and smaller_y <= possible_y_value2 <= bigger_y))\
            or ((obstacle_integer_coordinates[0] <= possible_x_value1 <= obstacle_integer_coordinates[0] + 1 and smaller_x <= possible_x_value1 <= bigger_x) or (obstacle_integer_coordinates[0] <= possible_x_value2 <= obstacle_integer_coordinates[0] + 1 and smaller_x <= possible_x_value2 <= bigger_x)):

            cut = True

    return cut

def is_cut(obstacles, center, node_to_go):

    is_cut = False

    for obstacle in obstacles:

        if is_cut_base(obstacle, center, node_to_go):

            is_cut = True

            break
    return is_cut

def turn_point(point_to_turn, turn_angle, parent):

    new_x = parent[0] + (point_to_turn[0] - parent[0]) * math.cos(turn_angle) - (
                point_to_turn[1] - parent[1]) * math.sin(turn_angle)

    new_y = parent[1] + (point_to_turn[0] - parent[0]) * math.sin(turn_angle) + (
                point_to_turn[1] - parent[1]) * math.cos(turn_angle)

    return [new_x,new_y]

def generate_random_angle():

    random_number = random.random()

    sign = 1 if random_number < 0.5 else -1

    magnitude = random.uniform(0, math.pi/6)

    angle = sign * magnitude

    return angle


def generate_individual():

    individual = Individual()

    number_of_genes = random.randint(0,50)

    for i in range(number_of_genes):

        individual.way_points.append(generate_random_angle())

    return individual


def generate_population():

    population = []

    for i in range(POPULATION_SIZE):

        population.append(generate_individual())

    return population


def evaluate_penalty_value(individual, launch_point_coordinates, target_point_coordinates, obstacle_points):
    distance_multiplier = 5
    obstacle_multiplier = 200
    target_miss_penalty = sys.maxsize
    if len(individual.way_points) == 0:

        if 3.1 <= eucledian_distance(launch_point_coordinates, target_point_coordinates) < 6.2:

            distance = eucledian_distance(launch_point_coordinates, target_point_coordinates)

            if not(is_cut(obstacle_points,launch_point_coordinates, target_point_coordinates)):

                return distance * distance_multiplier

            else:

                return distance * distance_multiplier + obstacle_multiplier

        else:

            return target_miss_penalty + sys.maxsize

    else:

        intersected_obstacle_counter = 0

        current_point = launch_point_coordinates

        next_point = find_point_on_circle(current_point, 3.1, individual.way_points[0])

        if is_cut(obstacle_points, current_point, next_point):

            intersected_obstacle_counter = intersected_obstacle_counter + 1

        current_point = next_point

        previous_point = launch_point_coordinates

        for i in range(1, len(individual.way_points)):

            straight = find_point_straight_on_circle(current_point, previous_point)

            next_point = turn_point(straight, individual.way_points[i], current_point)

            if is_cut(obstacle_points, current_point, next_point):

                intersected_obstacle_counter = intersected_obstacle_counter + 1

            previous_point = current_point

            current_point = next_point

        straight_point = find_point_straight_on_circle(current_point, previous_point)

        if 3.1 <= eucledian_distance(current_point, target_point_coordinates) < 6.2 and can_hit_target(current_point, straight_point, target_point_coordinates, math.pi/6):

            distance = ((len(individual.way_points) + 1)*3.1 + eucledian_distance(current_point, target_point_coordinates))*distance_multiplier

            if not (is_cut(obstacle_points, current_point, target_point_coordinates)):

                return intersected_obstacle_counter*obstacle_multiplier + distance*distance_multiplier

            else:
                intersected_obstacle_counter = intersected_obstacle_counter + 1
                return intersected_obstacle_counter*obstacle_multiplier + distance*distance_multiplier

        else:

            distance = ((len(individual.way_points) + 1) * 3.1 + eucledian_distance(current_point,target_point_coordinates)) * distance_multiplier

            if not (is_cut(obstacle_points, current_point, target_point_coordinates)):
                return target_miss_penalty + intersected_obstacle_counter * obstacle_multiplier + distance * distance_multiplier

            else:
                intersected_obstacle_counter = intersected_obstacle_counter + 1
                return target_miss_penalty + intersected_obstacle_counter * obstacle_multiplier + distance * distance_multiplier

def cross_over(individual1, individual2):

    half_index_of_first_individual = len(individual1.way_points)//2
    half_index_of_second_individual = len(individual2.way_points)//2

    new_ind1 = Individual()
    new_ind2 = Individual()

    new_ind1.way_points = individual2.way_points[:half_index_of_second_individual] + individual1.way_points[half_index_of_first_individual:]
    new_ind2.way_points = individual1.way_points[:half_index_of_first_individual] + individual2.way_points[half_index_of_second_individual:]

    return new_ind1, new_ind2


def mutation(individual,mutation_rate):

    rand_numb = random.randint(1, 1/mutation_rate)

    which_gene = random.randint(0, len(individual.way_points)-1)

    if rand_numb == 1:

        individual.way_points[which_gene] = generate_random_angle()


def cumulative_probability_array_generator(population):

    population.sort(key = lambda x: x.penalty_value)

    best_individual = population[0]

    fitness_array = []

    for i in population:

        fitness_array.append(1/i.penalty_value)


    sum_of_fitness_values = sum(fitness_array)

    for i in range(len(fitness_array)):

        fitness_array[i] = fitness_array[i]/sum_of_fitness_values

    cumulative_probability_array = []

    cumulative_probability_array.append(fitness_array[0])

    for i in range(1, len(fitness_array)):

        cumulative_probability_array.append(cumulative_probability_array[i-1] + fitness_array[i])

    return cumulative_probability_array

def turn_roulette_whell(population):

    rand_number = random.random()

    cumulative_probability_array = cumulative_probability_array_generator(population)

    chosen_individual = None

    for index in range(len(cumulative_probability_array)):

        if rand_number <= cumulative_probability_array[index]:

            chosen_individual = population[index]

            break

    return chosen_individual



def genetic_algorithm(launch, target, obstacles):

    bests_in_each_generation = []

    population = generate_population()

    for i in range(GENERATIONS):

        for individual in population:

            penalty_value = evaluate_penalty_value(individual,launch,target,obstacles)
            individual.penalty_value = penalty_value

        pairs_array = []

        pairs_counter = 0

        while not(pairs_counter == 100):

            person_counter = 0

            pair = []

            while not(person_counter == 2):

                if len(pair) == 0:
                    pair.append(turn_roulette_whell(population))
                    person_counter = person_counter + 1

                else:

                    chosen_individual = turn_roulette_whell(population)
                    if pair[0] is not chosen_individual:
                        pair.append(chosen_individual)
                        person_counter = person_counter + 1
            pairs_array.append(pair)

            pairs_counter = pairs_counter + 1

        best_indiviual = population[0]
        bests_in_each_generation.append(best_indiviual)
        population = []

        for pairs in pairs_array:

            new_ind1, new_ind2 = cross_over(pairs[0], pairs[1])

            population.append(new_ind1)
            population.append(new_ind2)

        for individual in population:

            mutation(individual, MUTPB)

    return bests_in_each_generation




#genetic_algorithm()