import numpy as np
import random


class RRT:
    def __init__(self, threshold=0.1):
        self.tree = {}
        self.threshold = threshold

    def is_collision(self, target, obstacles):
        for obstacle in obstacles:
            if np.linalg.norm(np.array(target) - np.array(obstacle)) <= self.threshold:
                return True
        return False

    def find_nearest_point(self, point):
        nearest_point = None
        min_distance = float('inf')
        for existing_point in self.tree:
            distance = np.linalg.norm(np.array(existing_point) - np.array(point))
            if distance < min_distance:
                nearest_point = existing_point
                min_distance = distance
        return nearest_point

    def new_point(self, nearest_point, random_point, step_size):
        direction = np.array(random_point) - np.array(nearest_point)
        direction = direction / np.linalg.norm(direction)
        new_point = np.array(nearest_point) + step_size * direction
        return tuple(new_point.tolist())

    def rrt(self, start, goal, obstacles, max_iterations=100, step_size=0.1, target_tolerance=0.1):
        self.tree[start] = None

        iterations = 0
        while iterations < max_iterations:
            random_point = (random.uniform(0, 1), random.uniform(0, 1)) #################
            nearest_point = self.find_nearest_point(random_point)
            new_point = self.new_point(nearest_point, random_point, step_size)

            if not self.is_collision(new_point, obstacles):
                self.tree[new_point] = nearest_point
                if np.linalg.norm(np.array(new_point) - np.array(goal)) <= target_tolerance:
                    self.tree[goal] = new_point
                    return self.build_path(start, goal)
            iterations += 1

        return None

    def build_path(self, start, goal):
        path = []
        current_point = goal
        while current_point != start:
            path.insert(0, current_point)
            current_point = self.tree[current_point]
        path.insert(0, start)
        return path


