from constants import PI, FACTOR
from math import sqrt
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
from typing import Tuple


def pgm_to_image(file_path: str):
    with open(file_path, 'rb') as file:
        return plt.imread(file)


def euclidean_distance(coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
    c1x, c1y = coord1
    c2x, c2y = coord2
    return sqrt((c1x - c2x) ** 2 + (c1y - c2y) ** 2)


def heuristic_function(start: Tuple[float, float], end: Tuple[float, float]) -> float:
    return abs(start[0] - end[0]) + abs(start[1] - end[1])


def handle_inv_coords(x: float, y: float) -> Tuple[float, float]:
    return FACTOR * (10 - y), FACTOR * (10 + x)


def handle_coords(x: float, y: float) -> Tuple[float, float]:
    return (y / FACTOR) - 10, 10 - (x / FACTOR)


def normalize(angle: float) -> float:
    while angle > PI:
        angle -= 2 * PI
    while angle < -PI:
        angle += 2 * PI
    return angle


euler_quat = lambda var: euler_from_quaternion(var)
