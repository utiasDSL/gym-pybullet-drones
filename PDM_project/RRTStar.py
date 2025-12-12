import numpy as np
import random
import math

class RRTStar:
    
    # Initialize the RRT
    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=0.5, goal_sample_rate=10, max_iter=500):
        """
        start: [x, y, z]
        goal:  [x, y, z]
        obstacle_list: list of obstacles [[x,y,z,size], ...]
        rand_area: [min, max] for random sampling range
        """
        self.start = Node(start[0], start[1], start[2])
        self.goal = Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.search_radius = 2.0

    # Path plan, it returns a list of waypoints [x, y, z] form start to goal
    def plan(self):
        
        self.node_list = [self.start]
        
        for i in range(self.max_iter):

            # 1. Sample a random node
            rnd_node = self.get_random_node()
            
            # 2. Find nearest node in tree
            nearest_node = self.get_nearest_node(self.node_list, rnd_node)

            # 3. Steer towards it (move for expand_dis)
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            # 4. Check Collision
            if not self.check_collision(new_node, self.obstacle_list):
                neighbors = self.find_neighbors(new_node)
                new_node = self.choose_parent(neighbors, nearest_node, new_node)
                self.node_list.append(new_node)
                self.rewire(new_node, neighbors)

                # 5. Check if close to goal
                dist_to_goal = self.calc_dist_to_goal(new_node.x, new_node.y, new_node.z)
                if dist_to_goal <= self.expand_dis:
                    final_node = self.steer(new_node, self.goal, self.expand_dis)
                    if not self.check_collision(final_node, self.obstacle_list):
                        return self.generate_final_course(len(self.node_list) - 1)

        return None  # Failed to find path

    # Go in the direction of the new node 
    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y, from_node.z)
        d, theta, phi = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_z = [new_node.z]

        if extend_length > d:
            extend_length = d

        # Move in 3D direction
        new_node.x += extend_length * math.sin(phi) * math.cos(theta)
        new_node.y += extend_length * math.sin(phi) * math.sin(theta)
        new_node.z += extend_length * math.cos(phi)

        new_node.parent = from_node
        new_node.cost = from_node.cost + extend_length

        return new_node

    # Pick random node
    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(0.1, 2.0)                         # Assume height is between 0.1 and 2.0, will be 0, 10
            )
        else:  # Goal bias
            rnd = Node(self.goal.x, self.goal.y, self.goal.z)
        return rnd

    # Check for collisions with obstacles
    def check_collision(self, node, obstacle_list):
        """
        Checks if the node is inside any Box obstacle.
        
        obstacle_list format: [[center_x, center_y, center_z, half_x, half_y, half_z], ...]
        
        Example: A 1x1x1m cube at (0,0,0) would be: [0, 0, 0, 0.5, 0.5, 0.5]
        """
        if node is None:
            return False 
        for (ox, oy, oz, hx, hy, hz) in obstacle_list:
            if (ox - hx <= node.x <= ox + hx) and \
               (oy - hy <= node.y <= oy + hy) and \
               (oz - hz <= node.z <= oz + hz):
                return True
        return False

        """
        for (ox, oy, oz, hx, hy, hz) in obstacle_list:

            # Iterate through all points in the path (usually just one for basic RRT)
            for x, y, z in zip(node.path_x, node.path_y, node.path_z):
                
                # Check if point is inside the XYZ bounds of the box
                # We use a small margin (>= instead of >) to catch grazing collisions
                if (ox - hx <= x <= ox + hx) and \
                   (oy - hy <= y <= oy + hy) and \
                   (oz - hz <= z <= oz + hz):
                    return True  # Collision detected

        return False  # Safe
        """
    
    # Retrive the path from start to goal
    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y, self.goal.z]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.z])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.z])
        return path[::-1] # Return reversed path (start -> goal)

    # Calculate distance to goal
    def calc_dist_to_goal(self, x, y, z):
        return np.linalg.norm([x - self.goal.x, y - self.goal.y, z - self.goal.z])

    """
    # Retrive nearest node index
    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 + (node.z - rnd_node.z)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind
    """
    
    # Retrive nearest node
    def get_nearest_node(self, node_list, rnd_node):
        distances = []
        for node in node_list:
            d = np.linalg.norm([node.x - rnd_node.x, node.y - rnd_node.y, node.z - rnd_node.z])
            distances.append(d)
        nearest_node = node_list[np.argmin(distances)]
        return nearest_node

    # Calculate distance and agle to go from a node to the next
    def calc_distance_and_angle(self, from_node, to_node):
        d = np.linalg.norm([to_node.x - from_node.x, to_node.y - from_node.y, to_node.z - from_node.z])
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x) 
        phi = math.acos((to_node.z - from_node.z) / d) if d != 0 else 0
        return d, theta, phi
    
    # Find neighbors inside the search radius
    def find_neighbors(self, new_node):
        neighbors = []
        for node in self.node_list:
            dist = np.linalg.norm([node.x - new_node.x, node.y - new_node.y, node.z - new_node.z])  
            if dist < self.search_radius:
                neighbors.append(node)
        return neighbors

    # Chose new node's parent based on cost
    def choose_parent(self, neighbors, nearest_node, new_node):
        min_cost = nearest_node.cost + np.linalg.norm([new_node.x - nearest_node.x, new_node.y - nearest_node.y, new_node.z - nearest_node.z])
        best_node = nearest_node
        for neighbor in neighbors:
            dist = np.linalg.norm([new_node.x - neighbor.x, new_node.y - neighbor.y, new_node.z - neighbor.z])
            cost = neighbor.cost + dist
            if cost < min_cost:
                best_node = neighbor
                min_cost = cost
        new_node.cost = min_cost
        new_node.parent = best_node
        return new_node
    
    # Rewire the three by checking if any neighbor should adopt the new node as a parent
    def rewire (self, new_node, neighbors):
        for neighbor in neighbors:
            dist = np.linalg.norm([neighbor.x - new_node.x, neighbor.y - new_node.y, neighbor.z - new_node.z])
            cost = new_node.cost + dist
            if cost < neighbor.cost:
                neighbor.parent = new_node
                neighbor.cost = cost


class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.path_x = []
        self.path_y = []
        self.path_z = []
        self.parent = None
        self.cost = 0.0