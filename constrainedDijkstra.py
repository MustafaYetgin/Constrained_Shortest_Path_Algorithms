import math
import heapq
import sys
from decimal import Decimal
TOLERANCE =  1e-10
class Node:

    def __init__(self):
        self.row_val =-1
        self.column_val = -1
        self.parent = None
        self.distance_to_launch = 0

    def __init__(self, row_val, column_val, parent, distance):
        self.row_val = row_val
        self.column_val = column_val
        self.parent = parent
        self.distance_to_launch = distance

    def set_parent(self, parent):

        self.parent = parent

    def set_distance(self, distance):

        self.distance_to_launch = distance

    def __lt__(self, other):
        return self.distance_to_launch < other.distance_to_launch


def eucledian_distance(current_coord, target_coord):
    return math.sqrt(math.pow(target_coord[1]-current_coord[1], 2)+math.pow(target_coord[0]-current_coord[0], 2))

def is_cut_base(obstacle_integer_coordinates, center, node_to_go):

    bigger_y, smaller_y = center.row_val, node_to_go.row_val
    bigger_x, smaller_x = center.column_val, node_to_go.column_val

    if center.row_val < node_to_go.row_val:
        bigger_y = node_to_go.row_val
        smaller_y = center.row_val

    if center.column_val < node_to_go.column_val:
        bigger_x = node_to_go.column_val
        smaller_x = center.column_val

    cut = False

    if abs(center.row_val - node_to_go.row_val) <= TOLERANCE:

        if obstacle_integer_coordinates[1] <= center.row_val <= obstacle_integer_coordinates[1] + 1: #or (abs(obstacle_integer_coordinates[1]-center.row_val) <= TOLERANCE or abs(obstacle_integer_coordinates[1]+1-center.row_val) <= TOLERANCE):

            if smaller_x <= obstacle_integer_coordinates[0] <= bigger_x or smaller_x <= obstacle_integer_coordinates[0]+1 <= bigger_x:

                cut = True

    elif abs(center.column_val - node_to_go.column_val) <= TOLERANCE:

        if obstacle_integer_coordinates[0] <= center.column_val <= obstacle_integer_coordinates[0] + 1:

            if smaller_y <= obstacle_integer_coordinates[1] <= bigger_y or smaller_y <= obstacle_integer_coordinates[1] + 1 <= bigger_y:
                cut = True

    else:

        slope = (node_to_go.row_val - center.row_val)/(node_to_go.column_val - center.column_val)
        b_value = center.row_val - slope*center.column_val

        possible_y_value1 = slope*obstacle_integer_coordinates[0]+b_value
        possible_y_value2 = slope*(obstacle_integer_coordinates[0]+1)+b_value

        possible_x_value1 = (obstacle_integer_coordinates[1] - b_value) / slope
        possible_x_value2 = (obstacle_integer_coordinates[1] + 1 - b_value) / slope

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

def is_put(point, open, search_sensitiity):

    putable = True
    for node in open:
        if eucledian_distance([node.column_val,node.row_val],[point.column_val,point.row_val]) <= search_sensitiity:
            putable = False
            break
    return putable


def add_or_subtract(point,slope):
    if slope < 0:
        if point.parent.row_val > point.row_val:
            return 1, -1
        else:
            return -1, 1
    elif slope>0:
        if point.parent.row_val > point.row_val:
            return -1,-1
        else:
            return 1,1
    else:
        if point.parent.column_val > point.column_val:
            return -1,0
        else:
            return 1,0

def turn_point(node, turn_angle, parent):

    new_x = parent.column_val + (node.column_val - parent.column_val) * math.cos(turn_angle) - (
                node.row_val - parent.row_val) * math.sin(turn_angle)

    new_y = parent.row_val + (node.column_val - parent.column_val) * math.sin(turn_angle) + (
                node.row_val - parent.row_val) * math.cos(turn_angle)

    return Node(new_y, new_x, parent, parent.distance_to_launch + 3.1)

def can_hit_target(center, straight_point,target ,max_angle_deg):

    center_to_point = [straight_point.column_val-center.column_val, straight_point.row_val-center.row_val]
    center_to_target = [target.column_val-center.column_val, target.row_val-center.row_val]

    magnitude_center_to_point = eucledian_distance([center.column_val,center.row_val],[straight_point.column_val,straight_point.row_val])
    magnitude_center_to_target = eucledian_distance([center.column_val, center.row_val],[target.column_val, target.row_val])

    dot_product = center_to_point[0] * center_to_target[0] + center_to_point[1] * center_to_target[1]

    cos_angle = dot_product / (magnitude_center_to_point*magnitude_center_to_target)

    angle = math.acos(cos_angle)

    return -max_angle_deg <= angle <= max_angle_deg

def find_point_straight_on_circle(center):

    if center.column_val - center.parent.column_val == 0:

         if center.row_val - center.parent.row_val < 0:
             return Node(center.row_val - 3.1, center.column_val, center, 3.1)

         elif center.row_val - center.parent.row_val > 0:
             return Node(center.row_val + 3.1, center.column_val, center, 3.1)

         else:
             return Node(center.row_val, center.column_val, center, 3.1)

    else:

        slope = (center.row_val - center.parent.row_val) / (center.column_val - center.parent.column_val)

        x_to_add = math.sqrt(math.pow(3.1, 2)/(math.pow(slope, 2)+1))

        y_to_add = abs(x_to_add*slope)

        mult1, mult2 = add_or_subtract(center, slope)

        x_val = center.column_val + x_to_add*mult1

        y_val = center.row_val + y_to_add*mult2

        return Node(y_val, x_val, center, 3.1)



def find_angle_with_x_axis(point, center):

    distance_from_center_in_x = abs(point.column_val - center.column_val)

    distance_from_center_in_y = abs(point.row_val - center.row_val)

    angle = math.atan2(distance_from_center_in_y, distance_from_center_in_x)

    return angle


def find_points_on_circle(center, radius, discretization_sensivity):

    cx, cy = center.column_val, center.row_val

    points = []

    for k in range(discretization_sensivity):

        angle = (2*math.pi*k) / discretization_sensivity

        x_axis = cx + radius*math.cos(angle)

        y_axis = cy + radius*math.sin(angle)

        points.append(Node(y_axis, x_axis, center, 3.1))

    return points


def does_contain(row, column, node_array):

    node_found = None

    for node in node_array:

        if node.row_val == row and node.column_val == column:

            node_found = node

            break

    return node_found

def find_points_on_circle_piece(center, discretization_sensivity):

    points = []

    point_straight = find_point_straight_on_circle(center)

    for k in range(discretization_sensivity+1):

        angle = (((math.pi/3)*k) / discretization_sensivity) - math.pi/6

        points.append(turn_point(point_straight, angle, center))

    return points


def constrained_dijkstra(launch_point_coordinate, target_coordinate, obstacles_coordinates, row_num, column_num, discretization_sensitivity ,searching_sensitivity):


    open = []

    closed = []

    noChild = []

    yesChild = []

    obstacle_cells = []

    road_found = False

    for x, y in obstacles_coordinates:



        if [int(x), int(y)] not in obstacle_cells:

            obstacle_cells.append([int(x),int(y)])

    target_node = Node(target_coordinate[1], target_coordinate[0], None, sys.maxsize)

    target_cell = [int(target_coordinate[0]), int(target_coordinate[1])]

    launch_node = Node(launch_point_coordinate[1], launch_point_coordinate[0], None, 0)

    if 3.1 <= eucledian_distance(launch_point_coordinate,target_coordinate) < 6.2 and not(is_cut(obstacle_cells,launch_node,target_node)):

        target_node.set_parent(launch_node)

        target_node.set_distance(eucledian_distance(launch_point_coordinate, target_coordinate))

        return target_node, [], [], closed, noChild, yesChild

    nodes = find_points_on_circle(launch_node, 3.1, 12)

    on_circle = nodes

    for node in nodes:

        if 0 <= node.row_val < row_num and 0 <= node.column_val < column_num:

            if does_contain(node.row_val, node.column_val, closed) is None:

                if not (is_cut(obstacle_cells, launch_node, node)) and not([int(node.column_val),int(node.row_val)] in obstacle_cells):

                    if [int(node.column_val), int(node.row_val)] == target_cell and eucledian_distance(launch_point_coordinate,target_coordinate) >= 3.1\
                            and not (is_cut(obstacle_cells, launch_node, target_node)):

                        target_node.set_parent(launch_node)

                        target_node.set_distance(eucledian_distance(launch_point_coordinate, target_coordinate))

                        return target_node, [], on_circle, closed, noChild, yesChild

                    heapq.heappush(open, node)

    heapq.heappush(closed, launch_node)

    heapq.heappush(open, target_node)

    all_opened = []

    while True:

        curr_opened = []

        closest_node = heapq.heappop(open)

        if closest_node is target_node:
            break

        straight_point = find_point_straight_on_circle(closest_node)

        if 3.1 <= eucledian_distance([closest_node.column_val, closest_node.row_val], target_coordinate) < 6.2 \
                and can_hit_target(closest_node, straight_point, target_node, math.pi / 6) \
                and not (is_cut(obstacle_cells, closest_node, target_node)) and not (is_cut(obstacle_cells, closest_node, target_node)) and not(target_coordinate in obstacles_coordinates):

            if target_node.distance_to_launch > eucledian_distance([closest_node.column_val, closest_node.row_val], target_coordinate) + closest_node.distance_to_launch:
                target_node.set_parent(closest_node)
                target_node.set_distance(closest_node.distance_to_launch + eucledian_distance([closest_node.column_val, closest_node.row_val], target_coordinate))

        nodelar = find_points_on_circle_piece(closest_node, discretization_sensitivity)

        for node in nodelar:

            if is_put(node, open, searching_sensitivity) and is_put(node, closed, searching_sensitivity):

                if does_contain(node.row_val, node.column_val, closed) is None:

                    if 0 <= node.row_val < row_num and 0 <= node.column_val < column_num:

                        if not (is_cut(obstacle_cells, closest_node, node)) and not([int(node.column_val),int(node.row_val)] in obstacle_cells):

                            if [int(node.column_val), int(node.row_val)] == target_cell \
                                    and eucledian_distance([closest_node.column_val, closest_node.row_val], target_coordinate) >= 3.1 \
                                    and can_hit_target(closest_node,straight_point,target_node,math.pi/6)\
                                    and not (is_cut(obstacle_cells, closest_node, target_node)):

                                if target_node.distance_to_launch > eucledian_distance([closest_node.column_val, closest_node.row_val],target_coordinate) + closest_node.distance_to_launch:
                                    target_node.set_parent(closest_node)
                                    target_node.set_distance(closest_node.distance_to_launch + eucledian_distance([closest_node.column_val, closest_node.row_val], target_coordinate))

                            curr_opened.append(node)

                            heapq.heappush(open, node)

        if len(curr_opened) == 0:
            noChild.append(closest_node)
        else:
            yesChild.append(closest_node)

        all_opened.append(curr_opened)

        heapq.heappush(closed, closest_node)

    return target_node, all_opened, on_circle,closed,noChild,yesChild, target_node.distance_to_launch

