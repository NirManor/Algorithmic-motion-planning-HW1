import argparse
import os
from typing import List, Tuple
import math

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString

def angle_between_lines(line1_points, line2_points):
    line1 = LineString(line1_points)
    line2 = LineString(line2_points)

    vector1 = (line1.xy[0][1] - line1.xy[0][0], line1.xy[1][1] - line1.xy[1][0])
    vector2 = (line2.xy[0][1] - line2.xy[0][0], line2.xy[1][1] - line2.xy[1][0])

    # Calculate the signed angle using atan2
    angle_rad = math.atan2(vector2[0], vector2[1]) - math.atan2(vector1[0], vector1[1])

    # Convert the angle to degrees
    angle_deg = math.degrees(angle_rad)

    # Ensure the angle is in the range [0, 360)
    angle_deg = (angle_deg + 360) % 360

    return angle_deg

def summing_polygon_vertices(v_1,v_2):
    v_x=v_1[0]+v_2[0]
    v_y=v_1[1]+v_2[1]
    return (v_x,v_y)


# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
    RobotCoords=[(0.0,-r),(r,0.0),(0.0,r),(-r,0.0),(0.0,-r)]

    #method 1 - convex hull (what is the time compexity of it)
    originalCoords = list(original_shape.exterior.coords)
    # Adding V_n+2 <- V_2 and W_n+2 <- W_2 to the polygon vertices List
    RobotCoords.append(RobotCoords[1])
    originalCoords.append(originalCoords[1])
    newCoords = []
    # method 1 - minkowski sum
    # for coord in originalCoords:
    #     newCoords.append(coord)
    #     newCoords.append((coord[0] - r, coord[1]))
    #     newCoords.append((coord[0] + r, coord[1]))
    #     newCoords.append((coord[0], coord[1] - r))
    #     newCoords.append((coord[0], coord[1] + r))
    # pointsForConvexHull = Polygon(newCoords)
    # convexHull = pointsForConvexHull.convex_hull

    #method 2 - minkowski sum algorithm
    # TODO?
    i=0
    j=0
    NewCoords=[]
    test=len(RobotCoords)-1
    while (i<len(RobotCoords)-1) and (j<len(originalCoords)-1):

        NewCoords.append(summing_polygon_vertices(RobotCoords[i],originalCoords[j]))
        angle_robot = angle_between_lines([RobotCoords[i],RobotCoords[i+1]], [(0, 0), (1, 0)])
        angle_obs = angle_between_lines([originalCoords[j],originalCoords[j+1]], [(0, 0), (1, 0)])
        if angle_robot<angle_obs:
            i+=1
        elif angle_robot>angle_obs:
            j+=1
        else:
            i+= 1
            j+= 1
    poly = Polygon(NewCoords)

    return poly


# TODO
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    pass


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # line1_points = [(0, 0), (1, 0)]
    # line2_points = [(0, 0.5), (-0.5, 0)]
    # test=angle_between_lines(line1_points, line2_points)

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    #TODO: fill in the next line
    shortest_path, cost = None, None

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))


    plotter3.show_graph()