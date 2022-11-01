from cProfile import label
from copy import deepcopy
from math import ceil, floor, sqrt
from typing import List, Tuple
from shapely.geometry import LinearRing, LineString, Polygon, Point
from matplotlib import pyplot as plt
import pymap3d
from random import randint

# Notes:
# Create points inside the area, but close to the original points, by adding the vectors from a given point to the two points next to it.
# For solving the general case, use the A* algorithm, but only on those inside points. Basically imagine drawing a line from the start waypoint
# to all other border_points and if it instersects the border on the way it's not possible, from there draw lines from where you are currently at,
# to the other border points, and update the path there if it is shorter than the one selected currently. After a while you will find the shortest path
# from the start waypoint to the end waypoint via any border_points that does not cross the border.


def euclidian_distance(
    point1: Tuple[float, float], point2: Tuple[float, float]
) -> float:
    return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def reduce_border_point(
    border_points: List[Tuple[float, float]],
    new_point_index: int,
    amount: float,
    inside: bool = True,
) -> Tuple[float, float]:
    if border_points[0] == border_points[-1]:
        border_points = border_points[:-1]

    old_point = border_points[new_point_index % len(border_points)]
    previous_old_point = border_points[(new_point_index - 1) % len(border_points)]
    next_old_point = border_points[(new_point_index + 1) % len(border_points)]

    previous_old_point_distance = euclidian_distance(old_point, previous_old_point)
    previous_old_vector = (
        (previous_old_point[0] - old_point[0]) / previous_old_point_distance,
        (previous_old_point[1] - old_point[1]) / previous_old_point_distance,
    )

    next_old_point_distance = euclidian_distance(old_point, next_old_point)
    next_old_vector = (
        (next_old_point[0] - old_point[0]) / next_old_point_distance,
        (next_old_point[1] - old_point[1]) / next_old_point_distance,
    )

    new_vector = (
        previous_old_vector[0] + next_old_vector[0],
        previous_old_vector[1] + next_old_vector[1],
    )

    new_vector_length = euclidian_distance(new_vector, (0, 0))

    new_vector = (
        new_vector[0] * amount / new_vector_length,
        new_vector[1] * amount / new_vector_length,
    )

    border = Polygon(border_points)

    new_point1 = (old_point[0] + new_vector[0], old_point[1] + new_vector[1])
    new_point2 = (old_point[0] - new_vector[0], old_point[1] - new_vector[1])

    if inside:
        if border.contains(Point(new_point1)):
            return new_point1
        elif border.contains(Point(new_point2)):
            return new_point2
    else:
        if not (border.contains(Point(new_point1))):
            return new_point1
        elif not (border.contains(Point(new_point2))):
            return new_point2

    print("Error in reduce_border_point! Returning point on the border")
    return old_point

def check_waypoints_on_border(
    border_points: List[Tuple[float, float]],
    waypoints: List[Tuple[float, float]],
    tolerance: float = 0.0001,
) -> bool:
    # border_linering = LinearRing(border_points)
    for waypoint in waypoints:
        # if border_linering.distance(Point(waypoint)) < tolerance:
        #     return True
        for border_point in border_points:
            if (
                abs(waypoint[0] - border_point[0]) < tolerance
                and abs(waypoint[1] - border_point[1]) < tolerance
            ):
                return True
    return False


def check_waypoints_in_border(
    border_points: List[Tuple[float, float]], waypoints: List[Tuple[float, float]]
) -> bool:
    border_polygon = Polygon(border_points)

    for waypoint in waypoints:
        waypoint = Point(waypoint)
        if not waypoint.within(border_polygon):
            return False
    return True


def check_waypoint_path_in_border(
    border_points: List[Tuple[float, float]], waypoints: List[Tuple[float, float]]
) -> bool:
    border_polygon = Polygon(border_points)
    waypoint_path = LineString(waypoints)
    return waypoint_path.within(border_polygon)


def path_within_border(
    border_points: List[Tuple[float, float]],
    waypoints: List[Tuple[float, float]],
    tolerance: float,
) -> List[Tuple[float, float]] | None:
    # if not check_waypoints_on_border(border_points, waypoints) and not check_waypoints_in_border(border_points, waypoints):
    #     return None

    if not check_waypoints_in_border(border_points, waypoints):
        return None

    if check_waypoint_path_in_border(border_points, waypoints):
        return waypoints

    border = LinearRing(border_points)

    conforming_waypoints = deepcopy(waypoints)

    number_of_added_points = 0
    for i in range(len(waypoints) - 1):
        path_segment = LineString(waypoints[i : i + 2])
        intersections = border.intersection(path_segment)

        if (
            intersections.is_empty
        ):  # or check_waypoints_on_border(border_points, waypoints[i:i+2]):
            continue

        intersection_indecies = []

        for j in range(len(border_points) - 1):
            fence_segment = LineString(border_points[j : j + 2])

            intersection = fence_segment.intersection(path_segment)

            if intersection.is_empty:
                continue

            intersection_indecies.append(j)
        
        if len(intersection_indecies) == 1:
            print("###############################\nError, only 1 index hit\n###############################")
            continue
        elif len(intersection_indecies) > 2:
            print("###############################\nError, more than 2 indicies hit\n###############################")

        index1, index2 = intersection_indecies
        additional_waypoints = []
        if index2 - index1 == 1:
            # additional_waypoints.append(border_points[index1+1])
            additional_waypoints.append(
                reduce_border_point(border_points, index1 + 1, tolerance)
            )
        if index2 - index1 == 2:
            # Here we have to decide which point will come first, choose the one closest to the start

            border_point1 = reduce_border_point(border_points, index1 + 1, tolerance)
            border_point2 = reduce_border_point(border_points, index1 + 2, tolerance)
            # border_point1 = border_points[index1+1]
            # border_point2 = border_points[index1+2]
            waypoint = waypoints[i]

            if euclidian_distance(border_point1, waypoint) <= euclidian_distance(
                border_point2, waypoint
            ):
                additional_waypoints.append(border_point1)
                additional_waypoints.append(border_point2)
            else:
                additional_waypoints.append(border_point2)
                additional_waypoints.append(border_point1)

        for j in range(len(additional_waypoints)):
            conforming_waypoints.insert(
                i + j + 1 + number_of_added_points, additional_waypoints[j]
            )

        number_of_added_points += len(additional_waypoints)

    return conforming_waypoints


def convert_gps_to_local_enu(
    lat_long_in: Tuple[float, float],
    height: float = 0, # The ground is about 43.3 meters above sea level,
    origin_lat_long: Tuple[float, float] = (38.31729702009844, -76.55617670782419),
) -> Tuple[float, float]:
    return pymap3d.geodetic2enu(lat_long_in[0], lat_long_in[1], height, origin_lat_long[0], origin_lat_long[1], height)[:2]

def convert_local_enu_to_gps(
    local_east_north_in: Tuple[float, float],
    height: float = 0, # The ground is about 43.3 meters above sea level,
    origin_lat_long: Tuple[float, float] = (38.31729702009844, -76.55617670782419),
) -> Tuple[float, float]:
    return pymap3d.enu2geodetic(local_east_north_in[0], local_east_north_in[1], height, origin_lat_long[0], origin_lat_long[1], height)[:2]

# Test the algorithm:

def generate_test_path(border_points: List[Tuple[float, float]], length_target: float = 16_000) -> List[Tuple[float, float]]:
    border = Polygon(border_points)
    waypoint_path_result = []

    x_points = [point[0] for point in border_points]
    y_points = [point[1] for point in border_points]
    max_x = floor(max(x_points))
    min_x = ceil(min(x_points))
    max_y = floor(max(y_points))
    min_y = ceil(min(y_points))



    while len(waypoint_path_result) < 2 or LineString(waypoint_path_result).length < length_target:
        new_point = (randint(min_x, max_x), randint(min_y, max_y))
        while not border.contains(Point(new_point)):
            new_point = (randint(min_x, max_x), randint(min_y, max_y))
        waypoint_path_result.append(new_point)
    
    return waypoint_path_result


def display(border_points_enu, waypoints_enu, conforming_path):
    plt.plot([point[0] for point in border_points_enu], [point[1] for point in border_points_enu], label="Border")
    plt.plot([point[0] for point in waypoints_enu], [point[1] for point in waypoints_enu], label="Original path")

    plt.plot([point[0] for point in conforming_path], [point[1] for point in conforming_path], label="Modified path")

    plt.legend()
    axes = plt.gca()
    axes.set_aspect(1)
    plt.show()


def main():
    # border_points = [(0, 0), (0, 2), (1,1), (2, 2), (2, 0), (1.1, 0.8), (0.9, 0.8), (0, 0)]
    # waypoints = [(0.3, 1.5), (1.8, 1.5)]
    # waypoints = [(1.8, 0.5), (0.3, 0.5)]

    border_points = [
        (38.31729702009844, -76.55617670782419),
        (38.31594832826572, -76.55657341657302),
        (38.31546739500083, -76.55376201277696),
        (38.31470980862425, -76.54936361414539),
        (38.31424154692598, -76.54662761646904),
        (38.31369801280048, -76.54342380058223),
        (38.31331079191371, -76.54109648475954),
        (38.31529941346197, -76.54052104837133),
        (38.31587643291039, -76.54361305817427),
        (38.31861642463319, -76.54538594175376),
        (38.31862683616554, -76.55206138505936),
        (38.31703471119464, -76.55244787859773),
        (38.31674255749409, -76.55294546866578),
        (38.31729702009844, -76.55617670782419),
    ]

    # border_points = [(point[1], point[0]) for point in border_points]

    # waypoints = [
    #     (38.31675, -76.55535),
    #     (38.318, -76.5515),
    #     (38.3167, -76.5446),
    #     (38.3153, -76.5418),
    # ]
    # waypoints = [(point[1], point[0]) for point in waypoints]

    border_points_enu = [convert_gps_to_local_enu(point) for point in border_points]
    # waypoints_enu = [convert_gps_to_local_enu(point) for point in waypoints]

    # for i in range(1000):
    #     print(f'{i}', end=" ")
    #     waypoints_enu = generate_test_path(border_points_enu)
    #     conforming_path = path_within_border(border_points_enu, waypoints_enu, 15)

    waypoints_enu = generate_test_path(border_points_enu)

    conforming_path = path_within_border(border_points_enu, waypoints_enu, 15)

    display(border_points_enu, waypoints_enu, conforming_path)

if __name__ == "__main__":
    main()