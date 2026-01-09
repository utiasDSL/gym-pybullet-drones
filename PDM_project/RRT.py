import numpy as np
import random
import math


# --- NODE CLASS DEFINITION ---
class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0


# --- RRT* CLASS DEFINITION ---
class RRTStar:

    def __init__(self, start, goal, obstacle_list, rand_area, 
                 expand_dis=3.0, goal_sample_rate=20, max_iter=500,
                 search_radius=None): 
        
        self.start = Node(start[0], start[1], start[2])
        self.goal = Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        
        # Search radius logic
        if search_radius is None:
            self.search_radius = self.expand_dis * 2.0 
        else:
            self.search_radius = search_radius

        self.resolution = 0.5   # Resolution for edge collision checking

    # Plan RRT* trajectory
    def plan(self):
        self.node_list = [self.start]
        
        for i in range(self.max_iter):

            # 1. Sample random node and get the nearest one
            rnd_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rnd_node)

            # 2. Steer towards the random node
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            # 3. Check segment collision between nearest and new node
            if not self.check_segment_collision(nearest_node, new_node, self.obstacle_list):

                # 4. Find neighbors and choose parent
                neighbors = self.find_neighbors(new_node)
                new_node = self.choose_parent(neighbors, nearest_node, new_node)
                self.node_list.append(new_node)

                # 5. Rewire 
                self.rewire(new_node, neighbors)

                # 6. Check Goal
                if self.calc_dist_to_goal(new_node.x, new_node.y, new_node.z) <= self.expand_dis:
                    final_node = self.steer(new_node, self.goal, self.expand_dis)

                    # Check if final segment to the goal is collision free
                    if not self.check_segment_collision(new_node, final_node, self.obstacle_list):
                        return self.generate_final_course(len(self.node_list) - 1)
        return None

    # Steer towards the new node
    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y, from_node.z)
        d, theta, phi = self.calc_distance_and_angle(new_node, to_node)

        if extend_length > d:
            extend_length = d

        new_node.x += extend_length * math.sin(phi) * math.cos(theta)
        new_node.y += extend_length * math.sin(phi) * math.sin(theta)
        new_node.z += extend_length * math.cos(phi)

        new_node.parent = from_node
        new_node.cost = from_node.cost + extend_length
        return new_node

    # Check if the segment betweem p1 and p2 is collision free (it is done every self.resolution meters)
    def check_segment_collision(self, p1, p2, obstacle_list):

        # Calculate distance between nodes
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if dist == 0: return False

        # Calculate number of steps needed
        steps = int(dist / self.resolution) + 1

        # Check every point along the line
        for i in range(steps + 1):
            t = i / steps
            x = p1.x + dx * t
            y = p1.y + dy * t
            z = p1.z + dz * t

            # Check if this specific point hits an obstacle and return True if collision detected
            if self.is_point_in_obstacle(x, y, z, obstacle_list):
                return True
        return False

    # Helper function to detect if a point is inside an obstacle
    def is_point_in_obstacle(self, x, y, z, obstacle_list):

        for (ox, oy, oz, hx, hy, hz) in obstacle_list:

            if (ox - hx <= x <= ox + hx) and \
               (oy - hy <= y <= oy + hy) and \
               (oz - hz <= z <= oz + hz):
                return True
        return False

    # Find nodes that are within a self.search_radius of new node
    def find_neighbors(self, new_node):
        neighbors = []

        for node in self.node_list:
            dist = np.linalg.norm([node.x - new_node.x, node.y - new_node.y, node.z - new_node.z])

            if dist < self.search_radius:
                neighbors.append(node)
        return neighbors
    
    # Choose the best parent (the one that minimizes the cost) among the neighbors of new node
    def choose_parent(self, neighbors, nearest_node, new_node):
        min_cost = nearest_node.cost + np.linalg.norm([new_node.x - nearest_node.x, new_node.y - nearest_node.y, new_node.z - nearest_node.z])
        best_node = nearest_node

        for neighbor in neighbors:
            dist = np.linalg.norm([new_node.x - neighbor.x, new_node.y - neighbor.y, new_node.z - neighbor.z])
            cost = neighbor.cost + dist

            # Check if neighbor minimizes the cost and the segment between it and new node is collision free
            if cost < min_cost and not self.check_segment_collision(neighbor, new_node, self.obstacle_list):
                min_cost = cost
                best_node = neighbor

        new_node.cost = min_cost
        new_node.parent = best_node
        return new_node

    # Rewire function
    def rewire(self, new_node, neighbors):

        for neighbor in neighbors:
            dist = np.linalg.norm([neighbor.x - new_node.x, neighbor.y - new_node.y, neighbor.z - new_node.z])
            cost = new_node.cost + dist

            # Check if neighbor minimizes the cost and the segment between it and new node is collision free
            if cost < neighbor.cost and not self.check_segment_collision(new_node, neighbor, self.obstacle_list):
                neighbor.parent = new_node
                neighbor.cost = cost

    # Calculate the nearest node
    def get_nearest_node(self, node_list, rnd_node):
        distances = []

        for node in node_list:
            d = np.linalg.norm([node.x - rnd_node.x, node.y - rnd_node.y, node.z - rnd_node.z])
            distances.append(d)

        nearest_node = node_list[np.argmin(distances)]
        return nearest_node

    # Pick a random node
    def get_random_node(self):

        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(0.1, 3.0)
            )
        else:
            rnd = Node(self.goal.x, self.goal.y, self.goal.z)

        return rnd

    # Computes final path
    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y, self.goal.z]]
        node = self.node_list[goal_ind]

        while node.parent is not None:
            path.append([node.x, node.y, node.z])
            node = node.parent

        path.append([self.start.x, self.start.y, self.start.z])
        return path[::-1]

    # Calculate the distance to the goal
    def calc_dist_to_goal(self, x, y, z):
        return np.linalg.norm([x - self.goal.x, y - self.goal.y, z - self.goal.z])

    # Calculate distance and angle to steer towards a node
    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        d = math.sqrt(dx**2 + dy**2 + dz**2)
        theta = math.atan2(dy, dx)
        phi = math.acos(max(-1.0, min(1.0, dz / d))) if d > 0 else 0
        return d, theta, phi