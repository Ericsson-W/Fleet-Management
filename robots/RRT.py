import numpy as np
import random
import matplotlib.pyplot as plt

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

    def rrt(self, start, goal, obstacles, max_iterations=1000, step_size=0.25, target_tolerance=1):
        print("RRT method called")
        start = tuple(start)
        goal = tuple(goal)
        self.tree[start] = None
        iterations = 0
        while iterations < max_iterations:
            print("RRT iteration:",iterations)
            random_point = (random.uniform(0.0, -3.01),random.uniform(0.0,3.01)) 
            nearest_point = self.find_nearest_point(random_point)
            new_point = self.new_point(nearest_point, random_point, step_size)

            if not self.is_collision(new_point, obstacles):
                self.tree[new_point] = nearest_point
                if np.linalg.norm(np.array(new_point) - np.array(goal)) <= target_tolerance:
                    self.tree[goal] = new_point
                    return self.build_path(start, goal)
            iterations += 1

            if iterations == max_iterations:
                print('max iterations reached')
                return None

    def build_path(self, start, goal):
        path = []
        current_point = goal
        while current_point != start:
            path.insert(0, current_point)
            current_point = self.tree[current_point]
        path.insert(0, start)
        return path

# rrt=RRT()

# start = (0,1.5)
# end= (-1.5,0)

# obstacle1 = (0.5, 0.5)
# obstacle2 = (-0.2, 0.6)
# obstacle3 = (-1,1.5)



# obstacles = [obstacle1, obstacle2, obstacle3]


# path = rrt.rrt(start, end, obstacles)

# if path:
#     print("Path Found",path)

# else:
#     print("path not found")

# plt.figure()
# plt.plot([start[0],end[0]],[start[1],end[1]],'go',label='plot')

# for obstacle in obstacles:
#     plt.plot(obstacle[0], obstacle[1], 'ro', label='Obstacle')

# if path:
#     path_x = [point[0] for point in path]
#     path_y = [point[1] for point in path]

#     plt.plot(path_x,path_y,'g',label='path')

# plt.xlabel('X')
# plt.ylabel('Y')
# plt.legend
# plt.title('RRT Obstacle avoidance')
# plt.grid()
# plt.show()
# plt.xlim[-3.01,0.0]
# plt.ylim[0.0,3.01]